#!/usr/bin/env python2

import math
from osgeo import osr


class CoordTransformer:
    def __init__(self, src_epsg=4326, tgt_epsg=4978):
        self.src = osr.SpatialReference()
        self.src.ImportFromEPSG(src_epsg)
        self.tgt = osr.SpatialReference()
        self.tgt.ImportFromEPSG(tgt_epsg)
        self.coord_tf = osr.CoordinateTransformation(self.src, self.tgt)

    def transform(self, fix):
        if fix is None:
            return None, None, None

        x, y, z = self.coord_tf.TransformPoint(fix.lon, fix.lat, fix.height)
        return x, y, z


class GGA:
    def __init__(self, msg):
        self.header = msg.header

        if not GGA.is_good_gga(msg.sentence):
            raise ValueError('Invalid GGA sentence passed')

        rest, self.checksum = msg.sentence.split('*')
        self.msg_id, self.utc, self.lat, self.lat_dir, \
        self.lon, self.lon_dir, self.quality, self.num_sat, \
        self.hdop, self.orthometric_height, \
        self.orthometric_height_unit, self. geoid_sep, \
        self.geoid_sep_unit, self.age, self.ref_station_id = rest.split(',')

        # 3820.9247885 -> 38 deg 20.924788' -> 38 + 20.924788 / 60 -> 38.348746475
        self.lat = float(self.lat)
        self.lat = math.floor(self.lat / 100) + (self.lat % 100) / 60
        self.lon = float(self.lon)
        self.lon = math.floor(self.lon / 100) + (self.lon % 100) / 60
        self.orthometric_height = float(self.orthometric_height)
        self.geoid_sep = float(self.geoid_sep)
        self.height = self.orthometric_height + self.geoid_sep

    @staticmethod
    def is_good_gga(gga, good_data_quality=None, bad_data_quality=None):
        if not gga or gga == '':
            return False
        if not gga.startswith('$GPGGA'):
            return False
        data_quality = gga.split(',')[6]
        if data_quality == '':
            return False
        if good_data_quality is not None:
            return data_quality in good_data_quality
        if bad_data_quality is not None:
            return data_quality not in bad_data_quality
        return True

