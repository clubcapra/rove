from typing import Dict, List
from diagnostic_msgs.msg import KeyValue


def dict2keyvalues(values: Dict[str, str]) -> List[KeyValue]:
    return list([KeyValue(key=k, value=v) for k, v in values.items()])


def yesno(value: bool) -> str:
    return 'yes' if value else 'no'
