# To use this code, make sure you
#
#     import json
#
# and then, to convert JSON from a string, do
#
#     result = odomdata_from_dict(json.loads(json_string))

from typing import Any, TypeVar, Type, cast


T = TypeVar("T")


def from_float(x: Any) -> float:
    assert isinstance(x, (float, int)) and not isinstance(x, bool)
    return float(x)


def to_float(x: Any) -> float:
    assert isinstance(x, float)
    return x


def from_int(x: Any) -> int:
    assert isinstance(x, int) and not isinstance(x, bool)
    return x


def from_str(x: Any) -> str:
    assert isinstance(x, str)
    return x


def to_class(c: Type[T], x: Any) -> dict:
    assert isinstance(x, c)
    return cast(Any, x).to_dict()


class Data:
    x: float
    y: float
    w: float

    def __init__(self, x: float, y: float, w: float) -> None:
        self.x = x
        self.y = y
        self.w = w

    @staticmethod
    def from_dict(obj: Any) -> 'Data':
        assert isinstance(obj, dict)
        x = from_float(obj.get("x"))
        y = from_float(obj.get("y"))
        w = from_float(obj.get("w"))
        return Data(x, y, w)

    def to_dict(self) -> dict:
        result: dict = {}
        result["x"] = to_float(self.x)
        result["y"] = to_float(self.y)
        result["w"] = to_float(self.w)
        return result


class Odomdata:
    type: int
    sc: str
    data: Data

    def __init__(self, type: int, sc: str, data: Data) -> None:
        self.type = type
        self.sc = sc
        self.data = data

    @staticmethod
    def from_dict(obj: Any) -> 'Odomdata':
        assert isinstance(obj, dict)
        type = from_int(obj.get("type"))
        sc = from_str(obj.get("sc"))
        data = Data.from_dict(obj.get("data"))
        return Odomdata(type, sc, data)

    def to_dict(self) -> dict:
        result: dict = {}
        result["type"] = from_int(self.type)
        result["sc"] = from_str(self.sc)
        result["data"] = to_class(Data, self.data)
        return result


def odomdata_from_dict(s: Any) -> Odomdata:
    return Odomdata.from_dict(s)


def odomdata_to_dict(x: Odomdata) -> Any:
    return to_class(Odomdata, x)
