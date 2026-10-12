# `testing`

**Not a colcon package.** No `package.xml`, no `setup.py`, no entry points — colcon skips
this directory entirely. These are standalone scripts you run with `python`, after
`conda activate robosub && source install/setup.bash`.

## Contents

| File | What it does |
|---|---|
| `testing_view.py` | Plotly **Dash** web app on `0.0.0.0:8052`. Discovers live topics, resolves their types with `rosidl_runtime_py.get_message`, and subscribes to whichever topic you pick in the browser. Publishes nothing. |
| `testing_view_old.py` | previous iteration, kept for reference |
| `dashboard_preview.html` | static mock-up of the panel layout |

```bash
python src/testing/testing_view.py        # then open http://<host>:8052
```

Extra Python deps beyond `requirements.txt`: `dash`, `plotly`.

## Related dashboards

Three separate surfaces exist; pick the right one:

| Surface | Port | Where |
|---|---|---|
| run dashboard (what you watch during a run) | 8080 | `hardware/sensors_plot` — see [`../hardware`](../hardware/README.md) |
| WebSocket HUD | 9091 + a static page | [`../gui`](../gui/README.md) |
| generic topic explorer | 8052 | this directory |
