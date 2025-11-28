from dummy_hand_control import DummyHandControl


# ホルダーの座標系は、ベース座標系（ツール座標系ではない）であることに注意
tool_infos = [
    {
        "id": 1,
        "name": "robotiq_epick",
        "tool_def": [0.0, 0.0, 296.0, 0.0, 0.0, 0.0],
    },
]
    
tool_classes = {
    "robotiq_epick": DummyHandControl,
}

tool_base = [360.03, 149.97, 460.03, -180, 0, -90]
