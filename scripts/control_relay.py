#!/usr/bin/env python3
"""Local WebSocket relay over dedicated SSH. Never repeats commands."""
import asyncio
from websockets.legacy.server import serve
from websockets.legacy.client import connect
async def relay(browser,path):
    try:
        async with connect('ws://127.0.0.1:18767',open_timeout=2,max_size=4096,max_queue=1,ping_interval=2,ping_timeout=2,close_timeout=.5) as vehicle:
            async def pipe(src,dst):
                async for message in src:await asyncio.wait_for(dst.send(message),.5)
            tasks=[asyncio.create_task(pipe(browser,vehicle)),asyncio.create_task(pipe(vehicle,browser))]
            try:await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
            finally:
                for task in tasks:task.cancel()
                await asyncio.gather(*tasks,return_exceptions=True)
    finally:await browser.close()
async def main():
    async with serve(relay,'127.0.0.1',8081,origins=['http://127.0.0.1:8080','http://localhost:8080'],max_size=4096,max_queue=1,close_timeout=.5):
        await asyncio.Future()
if __name__=='__main__':asyncio.run(main())
