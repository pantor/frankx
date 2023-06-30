#!/usr/bin/env python3

import pybind11_stubgen
import re


def add_union(alternative_type: str):
    def inner(match: re.Match, _alternative_type=alternative_type):
        return "typing.Union[{}, {}]".format(match.group(0), _alternative_type)

    return inner


if __name__ == '__main__':
    implicit_conversions = {
        "bool": "Condition",
        "float": "RelativeDynamicsFactor",
        "Affine": "RobotPose",
    }

    pybind11_stubgen.StubsGenerator.GLOBAL_CLASSNAME_REPLACEMENTS[
        re.compile("(Condition)")
    ] = add_union("bool")

    pybind11_stubgen.StubsGenerator.GLOBAL_CLASSNAME_REPLACEMENTS.update({
        re.compile("({})".format(orig_type)): add_union(alt_type)
        for alt_type, orig_type in implicit_conversions.items()
    })

    pybind11_stubgen.main()
