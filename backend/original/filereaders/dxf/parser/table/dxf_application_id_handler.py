
__author__ = 'Andreas Bachmann <andreas.bachmann@fablabwinti.ch>'

from filereaders.dxf.parser.table import dxf_table_handler


# Application ID (APPID) table
class DXFApplicationIdHandler(dxf_table_handler.DXFTableHandler):

    def __init__(self):
        from filereaders.dxf.dxf_constants import DXFConstants

        super(DXFApplicationIdHandler, self).__init__(DXFConstants.TABLE_TYPE_APPLICATION_ID)

    def parseGroup(self, groupCode, value):
        pass
