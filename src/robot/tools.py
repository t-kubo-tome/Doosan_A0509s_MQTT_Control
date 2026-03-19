from .config import HAND_IP
from .qbsofthand_industry_control import QbSoftHandIndustryControl


tool_infos = [
    {
        "id": 1,
        "name": "qb_soft_hand_industry",
        # NOTE: ユーザーマニュアルのCenter of Mass at fully openの値を使用しているが、
        # ツール先端の実測位置などを使用してもいいかもしれない
        "tool_def": [2.4, 14.3, 80.5, 0.0, 0.0, 0.0],
        "args": {
            "hand_ip": HAND_IP,
            "max_timeout": 10,
        }
    },
]
    
tool_classes = {
    "qb_soft_hand_industry": QbSoftHandIndustryControl,
}
