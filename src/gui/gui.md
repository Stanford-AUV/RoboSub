
**1. Start the WebSocket bridge**
```bash
ros2 run gui bridge
```

**2. Serve the HTML file**
```bash
cd src/gui
python3 -m http.server 8080
```

**3. On your laptop (VSCode)**

Forward both ports in the VSCode **Ports** panel:
- `8080` — HTML file server
- `9090` — WebSocket bridge

Then open in your browser: `http://localhost:8080/auv_hud.html`

**TODO**

YAML-ify auv_hud.html