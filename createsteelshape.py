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
        base:中间长
        x,y,z:两直角边形成的坡度以及斜边长
        length:钢筋总长
        diameter：钢筋直径
        N1 N5
        '''
        hooklength=(length-base-z*2)/2
        z+=diameter/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+x,-y)
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-x,-y)
        return profile,hooklength,135,135


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
        return profile,hooklength,135,135


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
        return profile,hooklength,135,135


    def shape_N2_steel(base=6860,length=7208,diameter=22):
        '''
        N2 N3 N3-1 N4 N44-2 N66-1 
        N10-1 N9-1 N10 N34-1 N44-3 
        N66-2 N14 N14-1 N14-2 N15 
        N19 N20 N35 N34-3 N31
        N66-3 N21-2 N21-3 N21-7
        N32 N34 N34-2 N39 N41
        N42 N43 N44 N44-1 N66-4
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+diameter/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-diameter/2,0)
        hooklength=(length-base)/2
        return profile,hooklength,-135,-135


    def shape_N2_1_steel(base=1525,down=400,length=2273,diameter=22):
        '''
        N2-1 N21-4 N22-1
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base+diameter/2,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,-down-diameter/2)
        hooklength=(length-base-down)/2
        return profile,hooklength,135,135


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
        return profile,hooklength,-135,-135


    def shape_N5_2_steel(base=7600,x=385,y=12,z=385,down=300,length=9256,diameter=18):
        '''
        N5-2
        '''
        hooklength=(length-base-down*2-z*2)/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        down+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+x,-y-down)
        profile+=AllplanGeo.Point3D(0,base/2+x,-y)
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-x,-y)
        profile+=AllplanGeo.Point3D(0,-base/2-x,-y-down)
        return profile,hooklength,135,135


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
        return profile,hooklength,135,135



    def shape_N7_steel(a=197,b=279,c=3776,base=1705,x=900,y=27,z=901,length=9832,diameter=18):
        '''
        N7 N8
        '''
        hooklength=143
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
        profile=AllplanGeo.Move(profile,AllplanGeo.Vector3D(0,0,-a))
        return profile,hooklength,135,135


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
        return profile,hooklength,135,135


    def shape_N14_3_steel(base=3310,down=180,length=3744,diameter=16):
        '''
        N14-3 N35-1
        '''
        hooklength=(length-base-down)/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,0,-down-diameter/2)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-base-diameter/2,0)
        return profile,hooklength,135,135

    def shape_N21_steel(length=32530,diameter=12):
        '''
        N21 N21-1 N22 N23 N24 N25 N26
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,-length/2,0)
        profile+=AllplanGeo.Point3D(0,length/2,0)
        return profile,-1,0,0


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
        return profile,hooklength,135,135


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
        return profile,hooklength,135,135


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
        return profile,hooklength,-135,-135


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
        profile+=AllplanGeo.Point3D(0,left*math.sin(leftangle),up-left*math.cos(leftangle))
        profile+=AllplanGeo.Point3D(0,left*math.sin(leftangle),-left*math.cos(leftangle))
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-right*math.sin(angle),right*math.cos(angle))
        return profile,hooklength,-135,-135



    def shape_N33_2_steel(up=180,left=1170,angle=81,length=1540,r=4,diameter=12):
        '''
        N33-2
        '''
        hooklength=(length-left-up)/2
        angle=angle/180*math.pi
        up+=diameter/2+r+1
        left+=diameter/2+r+1
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,left*math.sin(angle),left*math.cos(angle))
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,up)
        return profile,hooklength,-135,-135



    def shape_N38_steel(base=250,x=300,y=75,z=309,length=956,diameter=12):
        '''
        N38
        '''
        hooklength=(length-base-z*2)/2
        x,y=z/math.sqrt(x*x+y*y)*x,z/math.sqrt(x*x+y*y)*y
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,-x,y+base/2)
        profile+=AllplanGeo.Point3D(0,0,base/2)
        profile+=AllplanGeo.Point3D(0,0,-base/2)
        profile+=AllplanGeo.Point3D(0,-x,-y-base/2)
        return profile,-1,-135,-135



    def shape_N40_steel(left=1845,right=1851,angle=176,length=3886,diameter=12):
        '''
        N40
        '''
        hooklength=(length-left-right)/2
        angle=angle/180*math.pi
        left+=diameter/2
        right+=diameter/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,-left*math.cos(angle),-left*math.sin(angle))
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,-right,0)
        return profile,hooklength,135,135


    def shape_N45_steel(base=226,length=452,diameter=12):
        '''
        N45 N46 N47 N48 N48-1
        N49 N50 N51 N52 N53
        N54 N55 N56
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+diameter/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-diameter/2,0)
        hooklength=(length-base)/2
        return profile,hooklength,-135,-90


    def shape_N64_steel(base=112,down=156,length=524,diameter=8):
        '''
        N64
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2,-down-diameter/2)
        profile+=AllplanGeo.Point3D(0,base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2,-down-diameter/2)
        hooklength=(length-base-down*2)/2
        return profile,hooklength,180,180


    def shape_N65_steel(base=12530,length=12630,diameter=8):
        '''
        N65
        '''
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,base/2+diameter/2,0)
        profile+=AllplanGeo.Point3D(0,-base/2-diameter/2,0)
        hooklength=(length-base)/2
        return profile,hooklength,-180,-180


    def shape_N6_steel(a=337,b=5032,c=2840,d=2755,e=689,f=200,r=254,length=11786,diameter=20):
        '''
        N6 N6-1 N6-2 N12
        '''
        #todo modify
        hooklength=-1
        angle=math.atan(d/e)
        r=a/angle
        newangle=angle/2
        s=r*math.tan(newangle)  #求切线长
        b+=2*s
        c+=s
        d,e=c/math.sqrt(d*d+e*e)*d,c/math.sqrt(d*d+e*e)*e
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,b/2+e-f,d)
        profile+=AllplanGeo.Point3D(0,b/2+e,d)
        profile+=AllplanGeo.Point3D(0,b/2,0)
        profile+=AllplanGeo.Point3D(0,-b/2,0)
        profile+=AllplanGeo.Point3D(0,-b/2-e,d)
        profile+=AllplanGeo.Point3D(0,-b/2-e+f,d)
        return profile,-1,0,0


    def shape_N12_1_steel(a=338,base=1700,c=2841,d=2757,e=689,f=180,r=255,length=5382,up=180,diameter=18):
        '''
        N12-1
        '''
        #todo modify
        hooklength=length-a-base-c-f-up
        up+=diameter/2
        angle=math.atan(d/e)
        r=a/angle
        newangle=angle/2
        s=r*math.tan(newangle)  #求切线长
        base+=s
        c+=s
        d,e=c/math.sqrt(d*d+e*e)*d,c/math.sqrt(d*d+e*e)*e
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,0,up)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,base,0)
        profile+=AllplanGeo.Point3D(0,base+e,d)
        profile+=AllplanGeo.Point3D(0,base+e-f,d)
        return profile,hooklength,135,0


    def shape_N16_steel(xlength=890,width=561,length=3172,diameter=16):
        '''
        N16 N16-1 N17 N17-1 N18等钢筋形状
        xlength:箍筋横向长度
        width：箍筋宽度
        length:钢筋总长
        diameter：钢筋直径
        函数生成成型点
        '''
        hooklength=(length-xlength*2-width*2)/2
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,-hooklength,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,-width)
        profile+=AllplanGeo.Point3D(0,-xlength,-width)
        profile+=AllplanGeo.Point3D(0,-xlength,0)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,0,-hooklength)
        return profile,hooklength


    def shape_N9_steel(a=72,bottom=714,up=922,top=986,c=2996,d=2906,e=727,c1=2088,d1=2025,e1=506,f=200,r=54,length=8114,diameter=20):
        '''
        N9
        '''
        hooklength=(length-a-bottom-up-top-c-c1)/2
        angle=math.atan(d/e)
        r=a/angle
        newangle=angle/2
        s=r*math.tan(newangle)  #求切线长
        bottom+=s
        c+=s
        d,e=c/math.sqrt(d*d+e*e)*d,c/math.sqrt(d*d+e*e)*e
        c1+=hooklength
        d1,e1=c1/math.sqrt(d1*d1+e1*e1)*d1,c1/math.sqrt(d1*d1+e1*e1)*e1
        top+=hooklength
        profile = AllplanGeo.Polyline3D()
        profile+=AllplanGeo.Point3D(0,e1,d1+up)
        profile+=AllplanGeo.Point3D(0,0,up)
        profile+=AllplanGeo.Point3D()
        profile+=AllplanGeo.Point3D(0,bottom,0)
        profile+=AllplanGeo.Point3D(0,bottom+e,d)
        profile+=AllplanGeo.Point3D(0,bottom+e-top,d)
        return profile,-1,0,0


    shape_dic={
        'N1':shape_N1_steel,
        'N5':shape_N1_steel,
        'N1-1':shape_N1_1_steel,
        'N5-4':shape_N1_1_steel,
        'N1-2':shape_N1_2_steel,
        'N12-2':shape_N1_2_steel,
        'N35-2':shape_N1_2_steel,
        'N21-5':shape_N1_2_steel,
        'N21-6':shape_N1_2_steel,
        'N21-8':shape_N1_2_steel,
        'N21-9':shape_N1_2_steel,
        'N22-2':shape_N1_2_steel,
        'N22-3':shape_N1_2_steel,
        'N22-4':shape_N1_2_steel,
        'N2':shape_N2_steel,
        'N3':shape_N2_steel,
        'N3-1':shape_N2_steel,
        'N4':shape_N2_steel,
        'N44-2':shape_N2_steel,
        'N66-1':shape_N2_steel,
        'N10-1':shape_N2_steel,
        'N9-1':shape_N2_steel,
        'N10':shape_N2_steel,
        'N34-1':shape_N2_steel,
        'N44-3':shape_N2_steel,
        'N66-2':shape_N2_steel,
        'N14':shape_N2_steel,
        'N14-1':shape_N2_steel,
        'N14-2':shape_N2_steel,
        'N15':shape_N2_steel,
        'N19':shape_N2_steel,
        'N20':shape_N2_steel,
        'N35':shape_N2_steel,
        'N34-3':shape_N2_steel,
        'N31':shape_N2_steel,
        'N66-3':shape_N2_steel,
        'N21-2':shape_N2_steel,
        'N21-3':shape_N2_steel,
        'N21-7':shape_N2_steel,
        'N22-1':shape_N2_1_steel,
        'N32':shape_N2_steel,
        'N34':shape_N2_steel,
        'N34-2':shape_N2_steel,
        'N39':shape_N2_steel,
        'N41':shape_N2_steel,
        'N42':shape_N2_steel,
        'N45':shape_N2_steel,
        'N44':shape_N2_steel,
        'N44-1':shape_N2_steel,
        'N66-4':shape_N2_steel,
        'N2-1':shape_N2_1_steel,
        'N21-4':shape_N2_1_steel,
        'N2-2':shape_N2_2_steel,
        'N5-2':shape_N5_2_steel,
        'N5-3':shape_N5_3_steel,
        'N7':shape_N7_steel,
        'N8':shape_N7_steel,
        'N5-1':shape_N5_1_steel,
        'N14-3':shape_N14_3_steel,
        'N35-1':shape_N14_3_steel,
        'N21':shape_N21_steel,
        'N21-1':shape_N21_steel,
        'N22':shape_N21_steel,
        'N23':shape_N21_steel,
        'N24':shape_N21_steel,
        'N25':shape_N21_steel,
        'N26':shape_N21_steel,
        'N27':shape_N27_steel,
        'N28':shape_N27_steel,
        'N29':shape_N27_steel,
        'N30':shape_N27_steel,
        'N27-1':shape_N27_1_steel,
        'N33':shape_N33_steel,
        'N33-1':shape_N33_1_steel,
        'N33-2':shape_N33_2_steel,
        'N38':shape_N38_steel,
        'N40':shape_N40_steel,
        'N45':shape_N45_steel,
        'N46':shape_N45_steel,
        'N47':shape_N45_steel,
        'N48':shape_N45_steel,
        'N48-1':shape_N45_steel,
        'N49':shape_N45_steel,
        'N50':shape_N45_steel,
        'N51':shape_N45_steel,
        'N52':shape_N45_steel,
        'N53':shape_N45_steel,
        'N54':shape_N45_steel,
        'N55':shape_N45_steel,
        'N56':shape_N45_steel,
        'N64':shape_N64_steel,
        'N65':shape_N65_steel,
        'N6':shape_N6_steel,
        'N6-1':shape_N6_steel,
        'N6-2':shape_N6_steel,
        'N12':shape_N6_steel,
        'N16':shape_N16_steel,
        'N16-1':shape_N16_steel,
        'N17':shape_N16_steel,
        'N17-1':shape_N16_steel,
        'N18':shape_N16_steel,
        'N12-1':shape_N12_1_steel,
        'N9':shape_N9_steel
    }

    def get_shape_from_profile():
        return
        