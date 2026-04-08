# systemd Service Setup

## Install

Copy the service file to the systemd user directory (no sudo needed):

```bash
mkdir -p ~/.config/systemd/user
cp services/lidar_frame_parser.service ~/.config/systemd/user/
```

## Enable and Start

```bash
systemctl --user daemon-reload
systemctl --user enable lidar_frame_parser
systemctl --user start lidar_frame_parser
```

## Check Status / Logs

```bash
systemctl --user status lidar_frame_parser
journalctl --user -u lidar_frame_parser -f
```

## Stop / Disable

```bash
systemctl --user stop lidar_frame_parser
systemctl --user disable lidar_frame_parser
```

## Auto-start at boot (without login)

User services only start when you log in by default. To have it start at boot:

```bash
sudo loginctl enable-linger redux
```

## Notes

- Expects the binary at:
  `/home/redux/OcanathProj/code/lidar-frame-shaper/build/lidar_frame_parser`
- Build first with `cmake --build build` from the repo root if the binary is missing.
- The service restarts automatically on failure (5 second delay).
