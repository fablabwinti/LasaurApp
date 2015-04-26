
__author__ = 'Andreas Bachmann <andreas.bachmann@fablabwinti.ch>'

from filereaders.dxf.parser.table import dxf_table_handler


# Layer (LAYER) table
class DXFLayerHandler(dxf_table_handler.DXFTableHandler):

    def __init__(self):
        from filereaders.dxf.dxf_constants import DXFConstants

        super(DXFLayerHandler, self).__init__(DXFConstants.TABLE_TYPE_LAYER)

    def parseGroup(self, groupCode, value):
        pass
