"""
File Reader Module
"""


__author__ = 'Stefan Hechenberger <stefan@nortd.com>'


from .svg_reader import SVGReader
from .dxf_reader import DXFReader
from .ngc_reader import NGCReader
from .path_optimizers import optimize_all


def read_svg(svg_string, target_size, tolerance, forced_dpi=None, optimize=True):
    svgReader = SVGReader(tolerance, target_size)
    parse_results = svgReader.parse(svg_string, forced_dpi)
    if optimize:
        optimize_all(parse_results['boundarys'], tolerance)
    # {'boundarys':b, 'dpi':d, 'lasertags':l}
    return parse_results


def read_dxf(dxf_string, tolerance, optimize=True):
    dxfReader = DXFReader(tolerance)
    parse_results = dxfReader.parse(dxf_string)
    if optimize:
        optimize_all(parse_results['boundarys'], tolerance)
    # flip y-axis
    min_x = None
    min_y = None
    for color,paths in parse_results['boundarys'].items():
        for path in paths:
            for vertex in path:
                # FIXME: shouldn't hardcode the bedsize here
                #vertex[1] = 610.0 - vertex[1]
                vertex[1] = 575.0 - vertex[1]
                if min_x is None or vertex[0] < min_x:
                    min_x = vertex[0]
                if min_y is None or vertex[1] < min_y:
                    min_y = vertex[1]


    for color,paths in parse_results['boundarys'].items():
        for path in paths:
            for vertex in path:
                vertex[0] = vertex[0] - min_x
                vertex[1] = vertex[1] - min_y

    return parse_results


def read_ngc(ngc_string, tolerance, optimize=True):
    ngcReader = NGCReader(tolerance)
    parse_results = ngcReader.parse(ngc_string)
    # if optimize:
    #     optimize_all(parse_results['boundarys'], tolerance)
    return parse_results

