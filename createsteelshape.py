# -*- coding: utf-8 -*-
# @Time    : 2017/08/11
# @Author  : kingsley kuang
# @Site    : https://github.com/kingsley-gl/planbar.git
# @File    : LongitudinalBarShape.py ??Դ???ļ?
# @Software: 
# @Function: 

import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_Reinforcement as AllplanReinf
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import NemAll_Python_Utility as AllplanUtility          # allplan util library


import StdReinfShapeBuilder.GeneralReinfShapeBuilder as GeneralShapeBuilder
import StdReinfShapeBuilder.LinearBarPlacementBuilder as LinearBarBuilder


from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties as ConProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties as ReinforceProperties
from StdReinfShapeBuilder.RotationAngles import RotationAngles
import math

print("Loading Createsteelshape.py")



class Createsteelshape(object):
    
    def __init__(self,cover,rebar_prop):
        '''
        cover = [left,right,top,bottom]
        rebar_prop = [diameter,bending_roller,steel_grade,concrete_grade,bending_shape_type,mesh_type,mesh_bending_direction]
        '''
        if isinstance(cover, dict):
            self._cover = ConProperties(**cover)
        elif isinstance(cover, list):
            self._cover = ConProperties(*cover)
        else:
            self._cover = cover


        if isinstance(rebar_prop, dict):
            self._shape_props =  ReinforceProperties.rebar(**rebar_prop)
        elif isinstance(rebar_prop, list):
            self._shape_props =  ReinforceProperties.rebar(*rebar_prop)
        else:
            self._shape_props = rebar_prop

    def get_rotation_matrix_yz():
        """
        Get the rotation matrix for global/local transformation
        """

        rot_mat = AllplanGeo.Matrix3D()
        rot_angle = AllplanGeo.Angle()
        rot_angle.SetDeg(-90)
        rot_mat.Rotation(AllplanGeo.Line3D(AllplanGeo.Point3D(), AllplanGeo.Point3D(0, 1000, 0)), rot_angle)
        return rot_mat

    def shape_N1_steel(base=7600,x=900,y=27,z=901,length=9750,diameter=22):
        '''
        N1 N5
        '''
        hooklength=(length-base-z*2)/2
        x,y=(z+diameter/2)/math.sqrt(x*x+y*y)*x,(z+diameter/2)/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+x,-y)
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-x,-y)
        return profile,hooklength,135


    def shape_N1_1_steel(base=1895,x=900,y=27,z=901,down=400,length=3544,diameter=22):
        '''
        N1-1 N5-4
        '''
        x,y=(z+diameter/2)/math.sqrt(x*x+y*y)*x,(z+diameter/2)/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base+x,-y)
        profile+=AllplanGeo.Point3D(0,base,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,-down-diameter/2)
        hooklength=(length-base-z-down)/2
        return profile,hooklength,135


    def shape_N1_2_steel(base=3470,down=400,length=4618,diameter=22):
        '''
        N1-2 N12-2 N35-2 N21-5 N21-6
        N21-8 N21-9 N22-2 N22-3 N22-4
        
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2,-down-diameter/2)
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,-down-diameter/2)
        hooklength=(length-base-down*2)/2
        return profile,hooklength,135


    def shape_N2_steel(base=6860,length=7208,diameter=22):
        '''
        N2 N3 N3-1 N4 N44-2 N66-1 
        N10-1 N9-1 N10 N34-1 N44-3 
        N66-2 N14 N14-1 N14-2 N15 
        N19 N20 N35 N34-3 N31
        N66-3 N21-2 N21-3 N21-4 N21-7
        N22-1 N32
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+diameter/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-diameter/2,0)
        hooklength=(length-base)/2
        return profile,hooklength,-135


    def shape_N2_1_steel(base=1525,down=400,length=2273,diameter=22):
        '''
        N2-1 N21-4
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base+diameter/2,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,-down-diameter/2)
        hooklength=(length-base-down)/2
        return profile,hooklength,135


    def shape_N2_2_steel(base=4670,r=1055,angle=174,length=7128,diameter=22):
        '''
        N2-2
        '''
        newangle=angle/180*math.pi
        hooklength=(length-base-r*2)/2
        r+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2-r*math.cos(newangle),r*math.sin(newangle))
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2+r*math.cos(newangle),r*math.sin(newangle))
        return profile,hooklength,-135


    def shape_N5_2_steel(base=7600,x=385,y=12,z=385,down=300,length=9256,diameter=18):
        '''
        N5-2
        '''
        hooklength=(length-base-down*2-z*2)/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+x,-y-down-diameter/2)
        profile+=AllplanGeo.Point3D(0,base/2+x,-y)
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-x,-y)
        profile+=AllplanGeo.Point3D(0,-base/2-x,-y-down-diameter/2)
        return profile,hooklength,135


    def shape_N5_3_steel(base=355,leftdown=155,rightdown=300,x=1450,y=29,z=1450,length=2546,diameter=18):
        '''
        N5-3
        '''
        hooklength=(length-base-leftdown-rightdown-z)/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,0,-leftdown-diameter/2)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-base,0)
        profile+=AllplanGeo.Point3D(0,-base-x,-y)
        profile+=AllplanGeo.Point3D(0,-base-x,-y-rightdown-diameter/2)
        return profile,hooklength,135



    def shape_N7_steel(a=197,b=279,c=3776,base=1705,x=900,y=27,z=901,length=9832,diameter=18):
        '''
        N7 N8
        '''
        hooklength=(length-c-b*2-base*2-z*2)/2
        z+=diameter/2
        a=b/math.sqrt(2)
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,c/2+a+base+x,a-y)
        profile+=AllplanGeo.Point3D(0,c/2+a+base,a)
        profile+=AllplanGeo.Point3D(0,c/2+a,a)
        profile+=AllplanGeo.Point3D(0,c/2,0)
        profile+=AllplanGeo.Point3D(0,-c/2,0)
        profile+=AllplanGeo.Point3D(0,-c/2-a,a)
        profile+=AllplanGeo.Point3D(0,-c/2-a-base,a)
        profile+=AllplanGeo.Point3D(0,-c/2-a-base-x,a-y)
        return profile,hooklength,135


    def shape_N5_1_steel(base=355,x=2000,y=40,z=2001,down=155,length=2765,diameter=16):
        '''
        N5-1
        '''
        x,y=(z+diameter/2)/math.sqrt(x*x+y*y)*x,(z+diameter/2)/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,0,-down-diameter/2)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-base,0)
        profile+=AllplanGeo.Point3D(0,-base-x,-y)
        hooklength=(length-base-z-down)/2
        return profile,hooklength,135


    def shape_N14_3_steel(base=3310,down=180,length=3744,diameter=16):
        '''
        N14-3 N35-1
        '''
        hooklength=(length-base-down)/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,0,-down-diameter/2)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-base-diameter/2,0)
        return profile,hooklength,135



    def shape_N21_steel(length=7208,diameter=12):
        '''
        N21 N21-1 N22 N23 N24 N25 N26
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+diameter/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-diameter/2,0)
        return profile,hooklength,-135


    def shape_N27_steel(base=1465,x=3300,y=341,z=3318,length=4973,diameter=12):
        '''
        N27 N28 N29 N30
        '''
        hooklength=(length-base-z)/2
        z+=diameter/2
        base+=diameter/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-x,-y)
        return profile,hooklength,135


    def shape_N27_1_steel(base=1465,x=3055,y=316,z=3072,down=180,length=4907,diameter=12):
        '''
        N27-1
        '''
        hooklength=(length-base-z-down)/2
        down+=diameter/2
        base+=diameter/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-x,-y)
        profile+=AllplanGeo.Point3D(0,-x,-y-down)
        return profile,hooklength,135


    def shape_N33_steel(left=2735,leftangle=9,right=1645,angle=152,length=4570,diameter=12):
        '''
        N33
        '''
        hooklength=(length-left-right)/2
        rightangle=180-leftangle-angle
        leftangle=leftangle/180*math.pi
        rightangle=rightangle/180*math.pi
        left+=diameter/2
        right+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,left*math.cos(leftangle),left*math.sin(leftangle))
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-right*math.cos(rightangle),right*math.sin(rightangle))
        return profile,hooklength,-135


    def shape_N33_1_steel(up=180,left=1360,leftangle=99,right=1645,rightangle=152,length=3375,diameter=12):
        '''
        N33-1
        '''
        hooklength=(length-left-right-up)/2
        angle=rightangle-180+leftangle
        leftangle=leftangle/180*math.pi
        angle=angle/180*math.pi
        up+=diameter/2
        right+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,0,up)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-left*math.sin(leftangle),left*math.cos(leftangle))
        profile+=AllplanGeo.Point3D(0,-left*math.sin(leftangle)-right*math.sin(angle),left*math.cos(leftangle)+right*math.cos(angle))
        return profile,hooklength,-135



    def shape_N33_2_steel(up=180,left=1170,angle=81,length=1540,diameter=12):
        '''
        N33-2
        '''
        hooklength=(length-left-up)/2
        angle=angle/180*math.pi
        up+=diameter/2
        left+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,left*math.sin(angle),left*math.cos(angle))
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,up)
        return profile,hooklength,-135


    def shape_N33_3_steel(up=180,left=1170,angle=81,length=1540,diameter=12):
        '''
        N33-1
        '''
        hooklength=(length-left-up)/2
        angle=angle/180*math.pi
        up+=diameter/2
        left+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,left,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,up*math.cos(angle),up*math.sin(angle))
        return profile,hooklength,-135

 
