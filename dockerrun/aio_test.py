import asyncio as aio
from time import sleep

def to_coroutine(f):
    async def wrapper(*args, **kwargs):
        return f(*args, **kwargs)
    return wrapper

@to_coroutine
def longtime():
    for i in range(10000000000000):
        print(i)

async def test():
    await longtime()
    return 42

async def main():
    print(await aio.wait_for(test(), 1))
    # try:
    # except aio.TimeoutError:
    #     return 0
    
aio.run(main())