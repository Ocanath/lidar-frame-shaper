#include <cstdio>

#define TINYCSOCKET_IMPLEMENTATION

// Platform headers (must come before GL on Windows)
#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#endif

// tinycsocket (must come before SDL - SDL redefines main to SDL_main)
#include "tinycsocket.h"

// SDL2
#include <SDL.h>

// OpenGL
#if defined(__ANDROID__) || defined(IMGUI_IMPL_OPENGL_ES3)
#include <GLES3/gl3.h>
#else
#include <GL/gl.h>
#endif

// ImGui
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

// byte-stuffing
#include "cobs.h"
#include "PPP.h"

// dartt-protocol
#include "dartt.h"
#include "dartt_sync.h"
#include "checksum.h"

// App
#include "ui.h"
#include "dartt_init.h"
#include "plotting.h"

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include "dartt_mctl_params.h"
#include "motor.h"
#include "spooler_robot.h"

// Helper: case-insensitive extension check
static bool ends_with_ci(const std::string& str, const std::string& suffix) 
{
	if (suffix.size() > str.size()) 
	{
		return false;
	}
	std::string tail = str.substr(str.size() - suffix.size());
	std::transform(tail.begin(), tail.end(), tail.begin(), ::tolower);
	return tail == suffix;
}


double thresh_dbl(double in, double hi, double lo)
{
	if(in > hi)
	{
		return hi;
	}
	else if(in < lo)
	{
		return lo;
	}
	return in;
}

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	// Drag-and-drop state

	// Touch-as-mouse hint (needed for Android touch input via SDL_GetMouseState)
#ifdef __ANDROID__
	SDL_SetHint(SDL_HINT_MOUSE_TOUCH_EVENTS, "1");
#endif

	// Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
	{
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		return -1;
	}

	// GL attributes
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
#ifdef __ANDROID__
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

	// Create window
	Uint32 window_flags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
#ifdef __ANDROID__
	window_flags |= SDL_WINDOW_FULLSCREEN;
#endif
	SDL_Window* window = SDL_CreateWindow(
		"DARTT Dashboard",
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		1280, 720,
		window_flags
	);
	
	if (!window) 
	{
		printf("Window creation failed: %s\n", SDL_GetError());
		return -1;
	}

	// Create GL context
	SDL_GLContext gl_context = SDL_GL_CreateContext(window);
	if (!gl_context) 
	{
		printf("GL context creation failed: %s\n", SDL_GetError());
		return -1;
	}
	SDL_GL_MakeCurrent(window, gl_context);
	SDL_GL_SetSwapInterval(1); // VSync

	// Initialize ImGui
	if (!init_imgui(window, gl_context)) 
	{
		printf("ImGui initialization failed\n");
		return -1;
	}

	Plotter plot;

	int width = 0;
	int height = 0;
	SDL_GetWindowSize(window, &width, &height);
	plot.init(width, height);

	if (tcs_lib_init() != TCS_SUCCESS)
	{
		printf("Failed to initialize tinycsocket\n");
	}
	else
	{
		printf("Initialize tinycsocket library success\n");
	}

	SpoolerRobot robot;
	robot.motors.reserve(2);
	robot.add_motor(0x1, "192.168.0.25", 5400);
	robot.add_motor(0x0, "192.168.0.26", 5400);
	robot.targ = -10e3;
	robot.k = 0.5;
	robot.kd = 3.0;
	robot.tmax = 600;
	robot.prev_time = 0;
	robot.rom_degrees = -21000;


	int num_motors = (int)robot.motors.size();
	plot.lines.resize(num_motors);
	for (int i = 0; i < num_motors; i++)
	{
		plot.lines[i].xsource = &plot.sys_sec;
		plot.lines[i].ysource = &robot.iq[i];
		plot.lines[i].color   = template_colors[(i+1) % (sizeof(template_colors)/sizeof(rgb_t))];
	}
	int numkeys = 0;
	const uint8_t * keys = SDL_GetKeyboardState(&numkeys);

	// Main loop
	bool running = true;
	bool do_pctl = false;
	uint8_t prev_space_state = 0;
	
	int mode = FORCE_MODE;
	bool clicked = false;
	while (running)
	{
		// Poll events
		SDL_Event event;
		while (SDL_PollEvent(&event)) 
		{
			ImGui_ImplSDL2_ProcessEvent(&event);
			if (event.type == SDL_QUIT) 
			{
				running = false;
			}
			if (event.type == SDL_WINDOWEVENT &&
				event.window.event == SDL_WINDOWEVENT_CLOSE &&
				event.window.windowID == SDL_GetWindowID(window))
			{
				running = false;
			}
			if(event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_FINGERDOWN)
			{
				clicked = true;
			}
			if(event.type == SDL_MOUSEBUTTONUP || event.type == SDL_FINGERUP)
			{
				clicked = false;
			}
		}

		// Start ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();

		// --- Read ---
		bool comms_good = robot.read();

		// --- Controller (mouse → tensions) ---
		double t1 = 0, t2 = 0;
		
		if (comms_good)
		{
			int mouse_x, mouse_y;
			SDL_GetMouseState(&mouse_x, &mouse_y);
			int w, h;
			SDL_GetWindowSize(window, &w, &h);
			
			double xpos = ((float)mouse_x - w/2.f) / (w/2.f);

			SDL_PumpEvents();	//needed for keystrokes
			if(mode == FORCE_MODE)
			{
				do_pctl = false;
			}
			else
			{
				do_pctl = true;
			}
			
			if(do_pctl)
			{
				robot.targ = thresh_dbl(robot.targ, 0, robot.rom_degrees);
				if(mode == PCTL_CURSOR && clicked)
				{
					robot.targ = ((xpos+1.f)/2.f)*robot.rom_degrees;
				}
				float velocity = (robot.dp[0] - robot.dp[1]);
				float f = robot.k*(robot.targ - robot.p[0]) - robot.kd * velocity;
				if(f > 0)
				{
					t1 = f;
					t2 = 100;
					// t2 = 300 + robot.dp[1];
				}
				else if(f < 0)
				{
					t2 = -f;
					t1 = 100;
					// t1 = 300 + robot.dp[0];
				}
				else
				{
					t1 = 200;
					t2 = 200;
				}

			}
			else if(clicked)
			{
				if(xpos >  0.1) 
				{ 
					t1 = xpos*robot.tmax;  
					t2 = 100; 
				}
				else if (xpos < -0.1) 
				{ 
					t2 = -xpos*robot.tmax; 
					t1 = 100; 
				}
				else
				{ 
					t1 = 200;
					t2 = 200; 
				}
			}
			t1 = thresh_dbl(t1, robot.tmax, 100.);
			t2 = thresh_dbl(t2, robot.tmax, 100.);
		}
		robot.t[0] = t1;
		robot.t[1] = t2;

		// --- Write ---
		robot.write();

		SDL_GetWindowSize(window, &plot.window_width, &plot.window_height);
		for (int i = 0; i < (int)plot.lines.size(); i++)
		{
			plot.lines[i].yscale  =  (float)plot.window_height / (robot.tmax*1.1f);
			plot.lines[i].yoffset = -(float)plot.window_height / 3.f;
		}
		plot.sys_sec = (float)(((double)SDL_GetTicks64())/1000.);

		//add new frame of data to each line, as determined by UI
		for(int i = 0; i < plot.lines.size(); i++)
		{
			plot.lines[i].enqueue_data(plot.window_width);
		}
		
		
		if(robot.do_oscillation)
		{
			robot.oscillate(plot.sys_sec);
		}

		// Render
		render_socket_ui(robot);
		render_telemetry_ui(robot, mode);
		ImGui::Render();
		int display_w, display_h;
		SDL_GetWindowSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		plot.render();	//must position here
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		SDL_GL_SwapWindow(window);
	}

	// Save UI settings back to config
	// save_dartt_config("config.json", config);

	// Cleanup — teardown GL resources before destroying the context
	plot.teardown_gl_resources();
	shutdown_imgui();
	SDL_GL_DeleteContext(gl_context);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}
