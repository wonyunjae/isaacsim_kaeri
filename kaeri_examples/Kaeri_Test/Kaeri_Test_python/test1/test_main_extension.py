import asyncio
import os

import omni.ui as ui
from Kaeri_Test_python.kaeri_base_sample import BaseSampleExtension
from Kaeri_Test_python.test1 import Main

class MainExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="Test Core API",
            title="Testing Core API",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html",
            overview="This Example introduces the user on how to do cool stuff with Isaac Sim through scripting in asynchronous mode.",
            file_path=os.path.abspath(__file__),
            sample=Main(),
        )
        return
