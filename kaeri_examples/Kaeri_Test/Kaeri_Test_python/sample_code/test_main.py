import os
import sys

# 환경 변수 지정해줘야 함.
PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/isaac-sim/isaac_sim"
sys.path.append("/isaac-sim/isaac_sim")


from Kaeri_Test_python.kaeri_base_sample import BaseSample

class Main(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        
        return  

    def setup_scene(self):

        return

    async def setup_post_load(self):
        
        return

    # Articulation 이후에 한 번의 리셋이 필요하기 때문에 여기에 joints와 omnigraph가 있어야 함.
    async def setup_pre_reset(self):

        return

    async def setup_post_reset(self):

        return

    def world_cleanup(self):
        
        return