import asyncio
import websockets
from simulation.bridge import Bridge, Websocket, Direction

async def main():
    try:
        async with websockets.connect("ws://0.0.0.0:8765") as ws:
            websocket = Websocket(send=ws.send, recv=ws.recv)
            bridge = Bridge(out_direction=Direction.LOCAL_TO_DOCKER, in_direction=Direction.DOCKER_TO_LOCAL, ws=websocket)
            print("Bridge running")
            await bridge.run()
    except asyncio.CancelledError:
        print("Bridge stopped")

if __name__ == "__main__":
    asyncio.run(main())
