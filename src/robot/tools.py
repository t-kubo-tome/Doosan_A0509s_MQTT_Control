"""ツール一覧。"""
from robot import config
from robot.qbsofthand_industry_control import QbSoftHandIndustryControl

# 以降に使用するツール情報を追加すること

tool_infos = [
    {
        "id": 1,
        "name": "qb_soft_hand_industry",
        # NOTE: ユーザーマニュアルのCenter of Mass at fully openの値を使用しているが、
        # ツール先端の実測位置などを使用してもいいかもしれない
        "tool_def": [2.4, 14.3, 80.5, 0.0, 0.0, 0.0],
        "args": {
            "hand_ip": config.hand_ip,
            "max_timeout": 10,
        }
    },
]
    
tool_classes = {
    "qb_soft_hand_industry": QbSoftHandIndustryControl,
}

# 以降は触らないこと
tool_ids = [tool_info["id"] for tool_info in tool_infos]
