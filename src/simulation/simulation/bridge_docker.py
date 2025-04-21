from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from gz.msgs10.clock_pb2 import Clock
import uvicorn

from simulation.bridge import Bridge, Websocket, Direction

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Docker Local bridge is running"}


@app.websocket("/")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    websocket = Websocket(send=websocket.send_text, recv=websocket.receive_text)
    bridge = Bridge(out_direction=Direction.DOCKER_TO_LOCAL, in_direction=Direction.LOCAL_TO_DOCKER, ws=websocket)
    await bridge.run()
    # while True:
    #     data = await websocket.receive_text()
    #     print(data)


if __name__ == "__main__":
    uvicorn.run("bridge_docker:app", host="0.0.0.0", port=8765, reload=True)
