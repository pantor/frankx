#!/usr/bin/env python3
from typing import Dict

from pybind11_stubgen import *
from pybind11_stubgen.structs import *


class CustomWriter(Writer):
    def __init__(self, implicit_conversions: Dict[str, str], stub_ext: str = "pyi"):
        super().__init__(stub_ext=stub_ext)
        self.implicit_conversions = {
            QualifiedName.from_str(k): QualifiedName.from_str(v) for k, v in implicit_conversions.items()
        }

    def _patch_function(self, function: Function):
        for argument in function.args:
            if argument.annotation is not None and argument.annotation.name in self.implicit_conversions:
                converted_type = ResolvedType(self.implicit_conversions[argument.annotation.name])
                argument.annotation = ResolvedType(
                    QualifiedName.from_str("typing.Union"), [argument.annotation, converted_type])

    def write_module(self, module: Module, printer: Printer, to: Path, sub_dir: Optional[Path] = None):
        for cls in module.classes:
            for method in cls.methods:
                self._patch_function(method.function)
            for prop in cls.properties:
                if prop.setter is not None:
                    self._patch_function(prop.setter)
        super().write_module(module, printer, to, sub_dir=sub_dir)


IMPLICIT_CONVERSIONS = {
    "bool": "Condition",
    "float": "RelativeDynamicsFactor",
    "Affine": "RobotPose",
}

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(name)s - [%(levelname)7s] %(message)s",
    )
    args = arg_parser().parse_args()

    parser = stub_parser_from_args(args)

    printer = Printer(invalid_expr_as_ellipses=not args.print_invalid_expressions_as_is)

    out_dir, sub_dir = to_output_and_subdir(
        output_dir=args.output_dir,
        module_name=args.module_name,
        root_suffix=args.root_suffix,
    )

    run(
        parser,
        printer,
        args.module_name,
        out_dir,
        sub_dir=sub_dir,
        dry_run=args.dry_run,
        writer=CustomWriter(IMPLICIT_CONVERSIONS, stub_ext=args.stub_extension),
    )
