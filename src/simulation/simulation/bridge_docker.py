import asyncio
from simulation.bridge import Bridge, Direction

async def main():
    try:
        bridge = Bridge(out_direction=Direction.DOCKER_TO_LOCAL, in_direction=Direction.LOCAL_TO_DOCKER)
        print("Bridge running")
        await bridge.run()
    except asyncio.CancelledError:
        print("Bridge stopped")

if __name__ == "__main__":
    asyncio.run(main())
