from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Optional as _Optional

DESCRIPTOR: _descriptor.FileDescriptor

class PackageAcceptanceReq(_message.Message):
    __slots__ = ("robot_id", "del_hub_id")
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    DEL_HUB_ID_FIELD_NUMBER: _ClassVar[int]
    robot_id: str
    del_hub_id: str
    def __init__(self, robot_id: _Optional[str] = ..., del_hub_id: _Optional[str] = ...) -> None: ...

class PackageAcceptanceResp(_message.Message):
    __slots__ = ("accepted",)
    ACCEPTED_FIELD_NUMBER: _ClassVar[int]
    accepted: bool
    def __init__(self, accepted: bool = ...) -> None: ...
