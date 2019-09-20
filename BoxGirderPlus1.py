"""
Example Script for BoxGirderPlus1
"""

import NemAll_Python_Geometry as AllplanGeo
import NemAll_Python_BaseElements as AllplanBaseElements
import NemAll_Python_BasisElements as AllplanBasisElements
import math
from StdReinfShapeBuilder.RotationAngles import RotationAngles
import StdReinfShapeBuilder.LinearBarPlacementBuilder as LinearBarBuilder
import NemAll_Python_Reinforcement as AllplanReinf
from PythonPart import View2D3D, PythonPart
from JunheModels.util.createsteelshape import Createsteelshape
import StdReinfShapeBuilder.ProfileReinfShapeBuilder as ProfileShapeBuilder
import StdReinfShapeBuilder.GeneralReinfShapeBuilder as GeneralShapeBuilder
from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties

print('Load BoxGirderPlus1.py successfully')


def check_allplan_version(build_ele, version):
    """
    Check the current Allplan version

    Args:
        build_ele: the building element.
        version:   the current Allplan version

    Returns:
        True/False if version is supported by this script
    """

    # Delete unused arguments
    del build_ele
    del version

    # Support all versions
    return True


def create_element(build_ele, doc):
    """
    Creation of element

    Args:
        build_ele: the building element.
        doc:       input document
    """
    element = BoxGirderPlus1(doc)

    return element.create(build_ele)


def move_handle(build_ele, handle_prop, input_pnt, doc):
    """
    Modify the element geometry by handles

    Args:
        build_ele:  the building element.
        handle_prop handle properties
        input_pnt:  input point
        doc:        input document
    """

    build_ele.change_property(handle_prop, input_pnt)

    element = BoxGirderPlus1(doc)

    return element.create(build_ele)


class BoxGirderPlus1():
    """
    Definition of class BoxGirderPlus1
    """

    def __init__(self, doc):
        """
        Initialisation of class Hui2

        Args:
            doc: input document
        """

        self.model_ele_list = []
        self.handle_list = None
        self.document = doc
        self.com_prop = AllplanBaseElements.CommonProperties()
        self.com_prop.GetGlobalProperties()

    def data_read(self, build_dict):
        '''
        读取build_dict中的数据复制到当前类下
        '''
        for key, value in build_dict.items():
            self.__dict__[key] = value

    def create(self, build_ele):
        """
        Create the elements

        Args:
            build_ele:  the building element.

        Returns:
            tuple  with created elements and handles.
        """
        self.data_read(build_ele.get_parameter_dict())
        polyhedron = self.create_geometry()
        spiral=self.create_spiral_reinforcement()
        reinforcement = self.create_reinforcement()
        polyhedron+=spiral
        views = [View2D3D(polyhedron)]
        pythonpart = PythonPart('BoxGirderPlus1', parameter_list=(build_ele.get_params_list()),
          hash_value=(build_ele.get_hash()),
          python_file=(build_ele.pyp_file_name),
          views=views,
          reinforcement=reinforcement,
          common_props=(self.com_prop))
        if self.py:
            self.model_ele_list = pythonpart.create()
        else:
            polyhedron+=reinforcement
            self.model_ele_list = polyhedron
        return (self.model_ele_list, self.handle_list)

    def translate(self, element, trans_vector):
        """
        element:AllplanGeo元素
        trans_vector：平移向量
        根据元素和平移向量生成平移后的元素
        Translate element by translation vector
        """
        matrix = AllplanGeo.Matrix3D()
        matrix.Translate(trans_vector)
        return AllplanGeo.Transform(element, matrix)

    def reflection_barplacement(self,element,linear,mattype):
        '''
        element:Barplacement
        linear:是否线性铺设
        mattype:为0，关于梁体中心镜像；为1，关于梁跨中心镜像
        得到Barplacement的镜像
        '''
        matmg=AllplanGeo.Matrix3D()
        if mattype==0:
            matmg.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        elif mattype==1:
            matmg.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(self.halflength, 0, 0),AllplanGeo.Point3D(self.halflength, 0, 1),AllplanGeo.Point3D(self.halflength, 1, 0)))
        else:
            return element
        positionNumber=element.GetPositionNumber()
        shape=element.GetBendingShape()
        shape.Transform(matmg)
        start_point=element.GetStartPoint()
        start_point=AllplanGeo.Transform(start_point,matmg)
        distance_Vec=element.GetDistanceVector()
        distance_Vec=AllplanGeo.Transform(distance_Vec,matmg)
        position_point=AllplanGeo.Move(start_point,distance_Vec)
        barcount=element.GetBarCount()
        veclength=distance_Vec.GetLength()
        diameter=shape.GetDiameter()
        endshape=element.GetEndBendingShape()
        endshape.Transform(matmg)
        if linear:
            newele=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                positionNumber, shape,
                start_point,
                position_point,
                -diameter/2, veclength, barcount,False)
        else:
            newele=AllplanReinf.BarPlacement(positionNumber,barcount,shape,endshape)
        return newele

    def get_combine_barplacement(self,barplacement,pnum,shape):
        '''
        barplacement:Barplacement(列表)
        pnum:生成的铺设编号
        shape:配套钢筋形状
        vector:配套钢筋
        根据已有Barplacement（列表）得到配套钢筋的Barplacement列表
        '''
        if not isinstance(barplacement, list):
            barplacement=[barplacement]
        bar=[]
        for i in range(0,len(barplacement)):
            start_point=barplacement[i].GetStartPoint()
            distance_Vec=barplacement[i].GetDistanceVector()
            position_point=AllplanGeo.Move(start_point,distance_Vec)
            barcount=barplacement[i].GetBarCount()
            veclength=distance_Vec.GetLength()
            diameter=shape.GetDiameter()
            newele=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    pnum, shape,
                    start_point,
                    position_point,
                    -diameter/2, veclength, barcount,True)
            bar.append(newele)
        return bar

    def create_geometry(self):
        """
        Create the element geometries

        Args:
            build_ele:  the building element.
        """
        #取值
        
        height=self.q10+self.q20+self.q30+self.q40+self.b0-self.a10*self.p10-self.a20*self.p20-self.a30*self.p30-self.a40*self.p40
        self.height=height
        halfwidth=self.a10+self.a20+self.a30+self.a40
        #外截面
        outersection,outermain,outerscale=self.create_common_outer_section_path()
        #内截面参数
        id1list=[self.id01,self.id11,self.id21,self.id31,self.id41,self.id51,self.id61,self.id71,self.id81,self.id91]
        id2list=[self.id02,self.id12,self.id22,self.id32,self.id42,self.id52,self.id62,self.id72,self.id82,self.id92]
        uplinenumlist=[self.uplinenum0,self.uplinenum1,self.uplinenum2,self.uplinenum3,self.uplinenum4,self.uplinenum5,
            self.uplinenum6,self.uplinenum7,self.uplinenum8,self.uplinenum9]
        ie1list=[self.ie01,self.ie11,self.ie21,self.ie31,self.ie41,self.ie51,self.ie61,self.ie71,self.ie81,self.ie91]
        in1list=[self.in01,self.in11,self.in21,self.in31,self.in41,self.in51,self.in61,self.in71,self.in81,self.in91]
        ie2list=[self.ie02,self.ie12,self.ie22,self.ie32,self.ie42,self.ie52,self.ie62,self.ie72,self.ie82,self.ie92]
        in2list=[self.in02,self.in12,self.in22,self.in32,self.in42,self.in52,self.in62,self.in72,self.in82,self.in92]
        ie3list=[self.ie03,self.ie13,self.ie23,self.ie33,self.ie43,self.ie53,self.ie63,self.ie73,self.ie83,self.ie93]
        in3list=[self.in03,self.in13,self.in23,self.in33,self.in43,self.in53,self.in63,self.in73,self.in83,self.in93]
        downlinenumlist=[self.downlinenum0,self.downlinenum1,self.downlinenum2,self.downlinenum3,self.downlinenum4,
            self.downlinenum5,self.downlinenum6,self.downlinenum7,self.downlinenum8,self.downlinenum9]
        if1list=[self.if01,self.if11,self.if21,self.if31,self.if41,self.if51,self.if61,self.if71,self.if81,self.if91]
        im1list=[self.im01,self.im11,self.im21,self.im31,self.im41,self.im51,self.im61,self.im71,self.im81,self.im91]
        if2list=[self.if02,self.if12,self.if22,self.if32,self.if42,self.if52,self.if62,self.if72,self.if82,self.if92]
        im2list=[self.im02,self.im12,self.im22,self.im32,self.im42,self.im52,self.im62,self.im72,self.im82,self.im92]
        if3list=[self.if03,self.if13,self.if23,self.if33,self.if43,self.if53,self.if63,self.if73,self.if83,self.if93]
        im3list=[self.im03,self.im13,self.im23,self.im33,self.im43,self.im53,self.im63,self.im73,self.im83,self.im93]

        ibo1list=[self.ibo01,self.ibo11,self.ibo21,self.ibo31,self.ibo41,self.ibo51,self.ibo61,self.ibo71,self.ibo81,self.ibo91]
        ibo2list=[self.ibo02,self.ibo12,self.ibo22,self.ibo32,self.ibo42,self.ibo52,self.ibo62,self.ibo72,self.ibo82,self.ibo92]
        icelist=[self.ice0,self.ice1,self.ice2,self.ice3,self.ice4,self.ice5,self.ice6,self.ice7,self.ice8,self.ice9]
        icflist=[self.icf0,self.icf1,self.icf2,self.icf3,self.icf4,self.icf5,self.icf6,self.icf7,self.icf8,self.icf9]

        secdislist=[0,self.dis01,self.dis02,self.dis03,self.dis04,self.dis05,self.dis06,self.dis07,self.dis08,self.dis09]
        secsalist=[self.sa01,self.sa12,self.sa23,self.sa34,self.sa45,self.sa56,self.sa67,self.sa78,self.sa89]
        
        halflength=secdislist[self.secnum-1]
        self.halflength=secdislist[self.secnum-1]
        self.halfwidth=halfwidth
        #截面相同数据更新
        for i in range(0,self.secnum-1):
            if secsalist[i]:
                id1list[i+1]=id1list[i]
                id2list[i+1]=id2list[i]
                uplinenumlist[i+1]=uplinenumlist[i]
                ie1list[i+1]=ie1list[i]
                in1list[i+1]=in1list[i]
                ie2list[i+1]=ie2list[i]
                in2list[i+1]=in2list[i]
                ie3list[i+1]=ie3list[i]
                in3list[i+1]=in3list[i]
                downlinenumlist[i+1]=downlinenumlist[i]
                if1list[i+1]=if1list[i]
                im1list[i+1]=im1list[i]
                if2list[i+1]=if2list[i]
                im2list[i+1]=im2list[i]
                if3list[i+1]=if3list[i]
                im3list[i+1]=im3list[i]
                ibo1list[i+1]=ibo1list[i]
                ibo2list[i+1]=ibo2list[i]
                icelist[i+1]=icelist[i]
                icflist[i+1]=icflist[i]
        #生成截面
        innersectionlist=[]
        downlist=[]
        midlist=[]
        uplist=[]
        charmferlist=[]
        i=0
        for i in range(0,self.secnum):
            innersect,down,mid,up,charmfer=self.create_common_inner_section_path(id1list[i],id2list[i],height,uplinenumlist[i],ie1list[i],
                in1list[i],ie2list[i],in2list[i],ie3list[i],in3list[i],downlinenumlist[i],if1list[i],im1list[i],if2list[i],im2list[i],
                if3list[i],im3list[i],ibo1list[i],icelist[i],ibo2list[i],icflist[i],secdislist[i])
            innersectionlist.append(innersect)
            downlist.append(down)
            midlist.append(mid)
            uplist.append(up)
            charmferlist.append(charmfer)
        self.uplist=uplist
        self.midlist=midlist
        self.downlist=downlist
        matxz=AllplanGeo.Matrix3D()
        matxz.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        inners=[]
        pathlistlist=[]
        errlist=[]
        #生成内截面间实体列表
        i=0
        for i in range(0,self.secnum-1):
            #生成路径
            pathlis=[]
            pathlis+=self.create_path_by_point_list(self.patht,downlist[i],downlist[i+1])
            pathlis+=self.create_path_by_point_list(self.patht,midlist[i],midlist[i+1])
            upplist=self.create_path_by_point_list(self.patht,uplist[i],uplist[i+1])
            upplist.reverse()
            pathlis+=upplist
            #路径去重
            pathlist=self.list_modify(pathlis)
            #另一半路径
            pathli=pathlist[1:-1]
            pathli.reverse()
            #路径叠加
            for j in range(0,len(pathli)):
                pathre=AllplanGeo.Transform(pathli[j],matxz)
                pathlist.append(pathre)
            err,innersection= AllplanGeo.CreateRailSweptBRep3D([innersectionlist[i],innersectionlist[i+1]],pathlist,True,False,False)
            errlist.append(err)
            pathlistlist.append(pathlist)
            inners.append(innersection)
        #生成外截面实体
        outrail=AllplanGeo.Line3D(0,0,0,secdislist[self.secnum-1],0,0)
        errout,outshp=AllplanGeo.CreateSweptBRep3D([outersection],outrail,True,False,None,0)
        #内截面实体合并
        errunionin=[]
        inshp=inners[0]
        i=1
        for i in range(1,len(inners)):
            err,inshp=AllplanGeo.MakeUnion(inshp,inners[i])
            errunionin.append(err)
        #求减生成半实体
        errsub,halfGirderPlus=AllplanGeo.MakeSubtraction(outshp,inshp)
        #挖槽
        err,ring=self.create_ring_runway(self.bottomholehalflength,self.bottomholeradius,self.id02)
        if not err:
            errsubc,halfGirderPlus=AllplanGeo.MakeSubtraction(halfGirderPlus,ring)
        #镜像生成另一半实体
        matgir=AllplanGeo.Matrix3D()
        matgir.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(secdislist[self.secnum-1], 0, 0),AllplanGeo.Point3D(secdislist[self.secnum-1], 0, 1),AllplanGeo.Point3D(secdislist[self.secnum-1], 1, 0)))
        halfGirderPlus1=AllplanGeo.Transform(halfGirderPlus,matgir)
        #合并
        errcomb,GirderPlus=AllplanGeo.MakeUnion(halfGirderPlus,halfGirderPlus1)
        #泄水孔
        swhcy=[]
        sidewaterholedistancelist=self.string_to_list(self.waxdis)
        if self.waterholeawayy==0:
            swhy=self.waydis
        else:
            swhy=halfwidth-self.waydis
        swhx=0
        self.wateraxislist=[]
        for i in range(0,self.waterholenum):
            swhx+=sidewaterholedistancelist[i]
            axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(swhx,swhy,0),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,0,1))
            swh=AllplanGeo.BRep3D.CreateCylinder(axis,self.waterholediameter,height)
            self.wateraxislist.append(axis)
            self.wateraxislist+=self.wateraxislist
            rswh=AllplanGeo.Transform(swh,matxz)
            swhcy.append(swh)
            swhcy.append(rswh)
        if self.waterholeline>2:
            middlewaterholedistancelist=self.string_to_list(self.wamxdis)
            mwhx=0
            if self.waterholemawayy==0:
                mwhy=self.wamydis
            else:
                mwhy=halfwidth-self.wamydis
            for i in range(0,self.waterholemnum):
                mwhx+=middlewaterholedistancelist[i]
                axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(mwhx,mwhy,height/2),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,0,1))
                mwh=AllplanGeo.BRep3D.CreateCylinder(axis,self.waterholediameter,height/2)
                swhcy.append(mwh)
                self.wateraxislist.append(axis)
        #挖孔
        for i in range(0,len(swhcy)):
            errsub,GirderPlus=AllplanGeo.MakeSubtraction(GirderPlus,swhcy[i])
        #通风孔
        swihcy=[]
        sidewaterholedistancelist=self.string_to_list(self.wixdis)
        swihx=0
        self.windaxislist=[]
        if self.windholeawayy==1:
            swihz=self.wiydis
        else:
            swihz=height-self.wiydis
        for i in range(0,self.windholenum):
            swihx+=sidewaterholedistancelist[i]
            axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(swihx,-halfwidth,swihz),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,1,0))
            swih=AllplanGeo.BRep3D.CreateCylinder(axis,self.windholediameter,2*halfwidth)
            swihcy.append(swih)
            self.windaxislist.append(axis)
        if self.windholeline>1:
            swihcy=[]
            sidewaterholedistancelist=self.string_to_list(self.wimxdis)
            swihx=0
            if self.windholemawayy==1:
                mwihz=self.wimydis
            else:
                mwihz=height-self.wimydis
            for i in range(0,self.windholemnum):
                swihx+=sidewaterholedistancelist[i]
                axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(swihx,-halfwidth,mwihz),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,1,0))
                mwih=AllplanGeo.BRep3D.CreateCylinder(axis,self.windholediameter,2*halfwidth)
                swihcy.append(mwih)
                self.windaxislist.append(axis)
        #挖孔
        for i in range(0,len(swihcy)):
            errsub,GirderPlus=AllplanGeo.MakeSubtraction(GirderPlus,swihcy[i])
        com_prop = AllplanBaseElements.CommonProperties()
        com_prop.GetGlobalProperties()
        #生成顶板底层中点列表和底板顶层中点列表、生成内截面倒角铺设线
        self.topmidlist=AllplanGeo.Polyline3D()
        self.bottommidlist=AllplanGeo.Polyline3D()
        self.barinupcharmlist=AllplanGeo.Polyline3D()
        self.barindowncharmlist=AllplanGeo.Polyline3D()
        self.angle1list=[]
        self.angle2list=[]
        for i in range(0,self.secnum):
            self.topmidlist+=uplist[i][0]
            self.bottommidlist+=downlist[i][0]
            inupcharmfer=charmferlist[i][1]
            indowncharmfer=charmferlist[i][0]
            if(inupcharmfer!=0):
                inupcharm,angle1=self.get_point_and_angle_from_offset_arc3d(inupcharmfer,35)
                self.barinupcharmlist+=inupcharm
                self.angle1list.append(angle1)
            if(indowncharmfer!=0):
                indowncharm,angle2=self.get_point_and_angle_from_offset_arc3d(indowncharmfer,35)
                self.barindowncharmlist+=indowncharm
                self.angle2list.append(angle2)
        line34=AllplanGeo.Line3D(downlist[self.secnum-1][2],downlist[self.secnum-1][1])
        vector34=self.line3d_to_vector3d(line34)
        angle=math.atan(vector34.Z/vector34.Y)
        vector=self.vector_on_slope(angle,0,-35)
        point=line34.GetCenterPoint()
        point=point+vector
        self.barindowncharmlist+=point
        self.angle2list.append(angle*180/math.pi)
        #生成顶板顶层中点列表和底板底层中点列表
        self.toptoplist=AllplanGeo.Line3D(0,0,height,2*halflength,0,height)
        self.bottombottomlist=AllplanGeo.Line3D(0,0,0,2*halflength,0,0)
        #生成铺设线的偏移后实际折线
        self.bartopmidlist=self.polyline3d_offset(self.topmidlist,30)
        self.barbottommidlist=self.polyline3d_offset(self.bottommidlist,-35)
        self.bartopmidlist=self.polyline3d_reverse_longthen(self.bartopmidlist)
        self.barbottommidlist=self.polyline3d_reverse_longthen(self.barbottommidlist)
        self.bartoptoplist=AllplanGeo.Line3D(0,0,height-50,2*halflength,0,height-50)
        self.barbottombottomlist=AllplanGeo.Line3D(0,0,50,2*halflength,0,50)
        #完整的铺设线（加梁跨中心镜像）
        self.bartoptoplist=self.line3d_to_polyline3d(self.bartoptoplist)
        self.barbottombottomlist=self.line3d_to_polyline3d(self.barbottombottomlist)
        self.barinupcharmlist=self.polyline3d_reverse_longthen(self.barinupcharmlist)
        self.barindowncharmlist=self.polyline3d_reverse_longthen(self.barindowncharmlist)

        #N33系铺设线
        line=AllplanGeo.Line3D(outermain[6])
        line3=AllplanGeo.Move(line,AllplanGeo.Vector3D(self.dis03,0,0))
        in3=AllplanGeo.Line3D(self.uplist[3][2],self.uplist[3][1])
        polyline3=self.get_polyline_from_lines(line3,in3)
        polyline3=self.polyline3d_offset(polyline3,-35-12/2,x=self.dis03,type=0)
        line4=AllplanGeo.Move(line,AllplanGeo.Vector3D(self.dis04,0,0))
        in4=AllplanGeo.Line3D(self.uplist[4][2],self.uplist[4][1])
        polyline4=self.get_polyline_from_lines(line4,in4)
        polyline4=self.polyline3d_offset(polyline4,-35-12/2,x=self.dis04,type=0)
        n33barline=AllplanGeo.Polyline3D()
        n33barline+=polyline3[1]
        n33barline+=polyline4[1]
        self.n33barline=self.polyline3d_reverse_longthen(n33barline)
        #N10 N15系铺设线
        line1=outermain[0]
        line2=outermain[8]
        line=outermain[7]
        newline,point=self.get_middle_of_line_intersected_in_lines(line,line1,line2)
        self.barn15=self.get_polyline_from_point(point)
        self.barn10=AllplanGeo.Polyline3D()
        for i in range(0,self.secnum):
            newline1=AllplanGeo.Move(line1,AllplanGeo.Vector3D(secdislist[i],0,0))
            newline2=AllplanGeo.Move(line2,AllplanGeo.Vector3D(secdislist[i],0,0))
            newline=AllplanGeo.Line3D(midlist[i][1],midlist[i][0])
            newline,newpoint=self.get_middle_of_line_intersected_in_lines(newline,newline1,newline2)
            self.barn10+=newpoint
        self.barn10=self.polyline3d_reverse_longthen(self.barn10)
        self.barn10=AllplanGeo.Move(self.barn10,AllplanGeo.Vector3D(0,55,0))
        #是否显示区域块
        
        model_ele_list=[]
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.topmidlist))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.bartopmidlist))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.bottommidlist))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.barbottommidlist))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.barinupcharmlist))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.n33barline))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.barn10))
        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,self.barn15))
        if not errcomb and GirderPlus.IsValid:
            model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,GirderPlus))
        if self.blockvis:
            for i in range(0,len(inners)):
                if not errlist[i] and inners[i].IsValid:
                    inner=self.translate(inners[i],AllplanGeo.Vector3D(1000*i,0,12000))
                    model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,inner))
        if self.secvis:
            for i in range(0,len(inners)):
                    errpl1,erinner1=AllplanGeo.CreatePlanarBRep3D(innersectionlist[i])
                    if not errpl1 and erinner1.IsValid:
                        erin1=self.translate(erinner1,AllplanGeo.Vector3D(1000*i,0,-12000))
                        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,erin1))
                    errpl2,erinner2=AllplanGeo.CreatePlanarBRep3D(innersectionlist[i+1])
                    if not errpl2 and erinner2.IsValid:
                        erin2=self.translate(erinner2,AllplanGeo.Vector3D(1000*i,0,-12000))
                        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,erin2))
                    for j in range(0,len(pathlistlist[i])):
                        newpa=pathlistlist[i][j]
                        ernewpa=self.translate(newpa,AllplanGeo.Vector3D(1000*i,0,-12000))
                        model_ele_list.append(AllplanBasisElements.ModelElement3D(com_prop,ernewpa))
        return model_ele_list

    def create_outer_section_path(self,a1,p1,a2,p2,a3,p3,a4,p4,b,b1,c1,q1,c2,q2,c3,q3,c4,q4,br1,cr1,cr2):
        """
        a1,p1:第一段横向长度及坡度，坡度向上为正
        a2,p2:第二段横向长度及坡度，坡度向上为正
        a3,p3:第三段横向长度及坡度，坡度向上为正
        a4,p4:第四段横向长度及坡度，坡度向上为正
        b:梁边缘厚度
        b1:流水槽圆心到梁边缘下端直线距离
        c1,q1:第一段横向长度及纵向长度，纵向长度向上为正
        c2,q2:第二段横向长度及纵向长度，纵向长度向上为正
        c3,q3:第三段横向长度及纵向长度，纵向长度向上为正
        c4,q4:第四段横向长度及纵向长度，纵向长度向上为正
        获取截面外轮廓
        """
        #总长宽
        half_width=a1+a2+a3+a4
        height=q1+q2+q3+q4+b-a1*p1-a2*p2-a3*p3-a4*p4
        #线性主点
        pt0=AllplanGeo.Point3D(0,0,height)
        pt1=pt0+AllplanGeo.Vector3D(0,a1,a1*p1)
        pt2=pt1+AllplanGeo.Vector3D(0,a2,a2*p2)
        pt3=pt2+AllplanGeo.Vector3D(0,a3,a3*p3)
        pt4=pt3+AllplanGeo.Vector3D(0,a4,a4*p4)
        pt5=pt4+AllplanGeo.Vector3D(0,0,-b)
        pt6=pt5+AllplanGeo.Vector3D(0,-c4,-q4)
        pt7=pt6+AllplanGeo.Vector3D(0,-c3,-q3)
        pt8=pt7+AllplanGeo.Vector3D(0,-c2,-q2)
        pt9=pt8+AllplanGeo.Vector3D(0,-c1,-q1)
        #主线：
        ln0=AllplanGeo.Line3D(pt0,pt1)
        ln1=AllplanGeo.Line3D(pt1,pt2)
        ln2=AllplanGeo.Line3D(pt2,pt3)
        ln3=AllplanGeo.Line3D(pt3,pt4)
        ln4=AllplanGeo.Line3D(pt4,pt5)
        ln5=AllplanGeo.Line3D(pt5,pt6)
        ln6=AllplanGeo.Line3D(pt6,pt7)
        ln7=AllplanGeo.Line3D(pt7,pt8)
        ln8=AllplanGeo.Line3D(pt8,pt9)
        #流水槽
        cc=pt5+AllplanGeo.Vector3D(0,-b1,-b1*q4/c4)
        lsc=AllplanGeo.Arc3D(cc,AllplanGeo.Vector3D(0,-1,0),AllplanGeo.Vector3D(-1,0,0),br1,br1,0,2*math.pi,False)
        bo,pt5list=AllplanGeo.IntersectionCalculus(ln5,lsc,10e-10,2)
        ln51=AllplanGeo.Line3D(pt5,pt5list[1])
        AllplanGeo.Arc3D.SetStartPoint(lsc,pt5list[1])
        AllplanGeo.Arc3D.SetEndPoint(lsc,pt5list[0])
        ln52=AllplanGeo.Line3D(pt5list[0],pt6)
        #CR2倒角
        err1,ln6,ln7,fil67=AllplanGeo.FilletCalculus3D.Calculate(ln6,ln7,cr2)
        #CR1倒角
        err2,ln7,ln8,fil78=AllplanGeo.FilletCalculus3D.Calculate(ln7,ln8,cr1)
        #连成路径
        lpath=AllplanGeo.Path3D()
        lpath+=ln0
        lpath+=ln1
        lpath+=ln2
        lpath+=ln3
        lpath+=ln4
        lpath+=ln51
        lpath+=lsc
        lpath+=ln52
        lpath+=ln6
        lpath+=fil67
        lpath+=ln7
        lpath+=fil78
        lpath+=ln8
        #XZ平面镜像
        matxz=AllplanGeo.Matrix3D()
        matxz.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        rpath=AllplanGeo.Transform(lpath,matxz)
        #合并路径
        rpath.Reverse()
        path=lpath
        path+=rpath
        mainline=[ln0,ln1,ln2,ln3,ln4,ln5,ln6,ln7,ln8]
        return (path,lpath,mainline)

    def create_common_outer_section_path(self):
        """
        a1,p1:第一段横向长度及坡度，坡度向上为正
        a2,p2:第二段横向长度及坡度，坡度向上为正
        a3,p3:第三段横向长度及坡度，坡度向上为正
        a4,p4:第四段横向长度及坡度，坡度向上为正
        b:梁边缘厚度
        b1:流水槽圆心到梁边缘下端直线距离
        c1,q1:第一段横向长度及纵向长度，纵向长度向上为正
        c2,q2:第二段横向长度及纵向长度，纵向长度向上为正
        c3,q3:第三段横向长度及纵向长度，纵向长度向上为正
        c4,q4:第四段横向长度及纵向长度，纵向长度向上为正
        根据build_ele数据获取截面外轮廓
        """
        #总长宽
        half_width=self.a10+self.a20+self.a30+self.a40
        height=self.q10+self.q20+self.q30+self.q40+self.b0-self.a10*self.p10-self.a20*self.p20-self.a30*self.p30-self.a40*self.p40
        self.c40=self.a10+self.a20+self.a30+self.a40-self.c10-self.c20-self.c30
        #线性主点
        pt0=AllplanGeo.Point3D(0,0,height)
        pt1=pt0+AllplanGeo.Vector3D(0,self.a10,self.a10*self.p10)
        pt2=pt1+AllplanGeo.Vector3D(0,self.a20,self.a20*self.p20)
        pt3=pt2+AllplanGeo.Vector3D(0,self.a30,self.a30*self.p30)
        pt4=pt3+AllplanGeo.Vector3D(0,self.a40,self.a40*self.p40)
        pt5=pt4+AllplanGeo.Vector3D(0,0,-self.b0)
        pt6=pt5+AllplanGeo.Vector3D(0,-self.c40,-self.q40)
        pt7=pt6+AllplanGeo.Vector3D(0,-self.c30,-self.q30)
        pt8=pt7+AllplanGeo.Vector3D(0,-self.c20,-self.q20)
        pt9=pt8+AllplanGeo.Vector3D(0,-self.c10,-self.q10)
        #主线：
        ln0=AllplanGeo.Line3D(pt0,pt1)
        ln1=AllplanGeo.Line3D(pt1,pt2)
        ln2=AllplanGeo.Line3D(pt2,pt3)
        ln3=AllplanGeo.Line3D(pt3,pt4)
        ln4=AllplanGeo.Line3D(pt4,pt5)
        ln5=AllplanGeo.Line3D(pt5,pt6)
        ln6=AllplanGeo.Line3D(pt6,pt7)
        ln7=AllplanGeo.Line3D(pt7,pt8)
        ln8=AllplanGeo.Line3D(pt8,pt9)
        #流水槽
        cc=pt5+AllplanGeo.Vector3D(0,-self.b10,-self.b10*self.q40/self.c40)
        lsc=AllplanGeo.Arc3D(cc,AllplanGeo.Vector3D(0,-1,0),AllplanGeo.Vector3D(-1,0,0),self.br0,self.br0,0,2*math.pi,False)
        bo,pt5list=AllplanGeo.IntersectionCalculus(ln5,lsc,10e-10,2)
        ln51=AllplanGeo.Line3D(pt5,pt5list[1])
        AllplanGeo.Arc3D.SetStartPoint(lsc,pt5list[1])
        AllplanGeo.Arc3D.SetEndPoint(lsc,pt5list[0])
        ln52=AllplanGeo.Line3D(pt5list[0],pt6)
        #CR2倒角
        err1,ln6,ln7,fil67=AllplanGeo.FilletCalculus3D.Calculate(ln6,ln7,self.cr20)
        #CR1倒角
        err2,ln7,ln8,fil78=AllplanGeo.FilletCalculus3D.Calculate(ln7,ln8,self.cr10)
        #连成路径
        lpath=AllplanGeo.Path3D()
        lpath+=ln0
        lpath+=ln1
        lpath+=ln2
        lpath+=ln3
        lpath+=ln4
        lpath+=ln51
        lpath+=lsc
        lpath+=ln52
        lpath+=ln6
        lpath+=fil67
        lpath+=ln7
        lpath+=fil78
        lpath+=ln8
        #XZ平面镜像
        matxz=AllplanGeo.Matrix3D()
        matxz.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        rpath=AllplanGeo.Transform(lpath,matxz)
        #合并路径
        rpath.Reverse()
        lpath+=rpath
        mainline=[ln0,ln1,ln2,ln3,ln4,ln5,ln6,ln7,ln8]
        scaleline=[ln0,ln1,ln2,ln3,ln4,ln51,lsc,ln52,ln6,fil67,ln7,fil78,ln8]
        return (lpath,mainline,scaleline)

    def create_common_inner_section_path(self,d1,d2,height,uplinenum,e1,n1,e2,n2,e3,n3,downlinenum,f1,m1,f2,m2,f3,m3,bo1,ce1,bo2,cf1,d):
        """
        获取截面内轮廓
        """

        #线性主点
        downptlist=[]
        pt0=AllplanGeo.Point3D(d,0,d2)
        pt1=pt0+AllplanGeo.Vector3D(0,f1,m1)
        downptlist.append(pt0)
        downptlist.append(pt1)
        if downlinenum>1:
            pt2=pt1+AllplanGeo.Vector3D(0,f2,m2)
            downptlist.append(pt2)
            if downlinenum>2:
                pt3=pt2+AllplanGeo.Vector3D(0,f3,m3)
                downptlist.append(pt3)

        upptlist=[]
        pt4=AllplanGeo.Point3D(d,0,height-d1)
        pt5=pt4+AllplanGeo.Vector3D(0,e1,n1)
        upptlist.append(pt4)
        upptlist.append(pt5)
        if uplinenum>1:
            pt6=pt5+AllplanGeo.Vector3D(0,e2,n2)
            upptlist.append(pt6)
            if uplinenum>2:
                pt7=pt6+AllplanGeo.Vector3D(0,e3,n3)
                upptlist.append(pt7)
        upptlist.reverse()
        #主线：
        downlinelist=[]
        for i in range(0,len(downptlist)-1):
            downlinelist.append(AllplanGeo.Line3D(downptlist[i],downptlist[i+1]))
        midline=AllplanGeo.Line3D(downptlist[-1],upptlist[0])
        uplinelist=[]
        for i in range(0,len(upptlist)-1):
            uplinelist.append(AllplanGeo.Line3D(upptlist[i],upptlist[i+1]))

        #路径延伸        
        lpath=AllplanGeo.Path3D()
        for i in range(0,len(downlinelist)-1):
            lpath+=downlinelist[i]
        #倒角圆弧列表
        charmferlist=[]
        #CF1倒角
        if bo2:
            err1,ln1,midline,fil0=AllplanGeo.FilletCalculus3D.Calculate(downlinelist[-1],midline,cf1)
            lpath+=ln1
            lpath+=fil0
            downlinelist[-1]=ln1
            downptlist[-1]=ln1.GetEndPoint()
            charmferlist.append(fil0)
        else:
            lpath+=downlinelist[-1]
            charmferlist.append(0)
        #CE1倒角
        if bo1:
            err2,midline,ln2,fil1=AllplanGeo.FilletCalculus3D.Calculate(midline,uplinelist[0],ce1)
            lpath+=midline
            lpath+=fil1
            lpath+=ln2
            uplinelist[0]=ln2
            upptlist[0]=ln2.GetStartPoint()
            charmferlist.append(fil1)
        else:
            lpath+=midline
            lpath+=uplinelist[0]
            charmferlist.append(0)
        for i in range(1,len(uplinelist)):
            lpath+=uplinelist[i]

        #上中下三部分特征点
        down=downptlist
        mid=[midline.GetStartPoint(),midline.GetEndPoint()]
        upptlist.reverse()
        up=upptlist
        #XZ平面镜像
        matxz=AllplanGeo.Matrix3D()
        matxz.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        rpath=AllplanGeo.Transform(lpath,matxz)
        #合并路径
        rpath.Reverse()
        lpath+=rpath
        return (lpath,down,mid,up,charmferlist)

    def create_path_by_point_list(self,type,pl1,pl2):
        '''
        pl1:路径起始点集合
        pl2:路径终止点集合
        type:路径对应方式 
            为1：点集依次对应，一点集多出的点全部由另一点集最后一个点对应
            为2：点集依次对应，一点集多出的点除最后一个点之外全部舍去
            为3：点集依次对应，一点集多出的点全部舍去
        根据路径起始点集合、路径终止点集合，选择适合的路径对应方式生成对应路径集合
        '''
        a=len(pl1)
        b=len(pl2)
        pathlist=[]
        p1=pl1[:]
        p2=pl2[:]
        if type==1:
            if a>b:
                for i in range(0,a-b):
                    p2.append(pl2[-1])
            elif a<b:
                for i in range(0,b-a):
                    p1.append(pl1[-1])
            for i in range(0,max(a,b)):
                pathlist.append(AllplanGeo.Line3D(p1[i],p2[i]))
        elif type==2:
            if a > b:
                pl10 = pl1[:b-1]
                pl10.append(pl1[-1])
                pl1=pl10
            elif a < b:
                pl20 = pl2[:a-1]
                pl20.append(pl2[-1])
                pl2=pl20
            for i in range(0, min(a, b)):
                pathlist.append(AllplanGeo.Line3D(pl1[i],pl2[i]))
        else:
            for i in range(0, min(a, b)):
                pathlist.append(AllplanGeo.Line3D(pl1[i],pl2[i]))
        return pathlist

    def list_modify(self,list):
        '''
        集合去除相邻相同的元素
        '''
        index=[]
        li=list[:]
        for i in range(0,len(list)-1):
            if list[i]==list[i+1]:
                index.append(i)
        if len(index)!=0:
            head=list[:index[0]]
            mid=[]
            for i in range(0,len(index)-1):
                mid+=list[index[i]+1:index[i+1]]
            tail=list[index[-1]+1:]
            li=head+mid+tail
        return li   

    def create_reinforcement(self):
        '''
        根据build_ele生成普通钢筋（环形钢筋除外）
        '''
        reinforcement=[]
        vertical_reinforcement=self.create_vertical_reinforcement()
        horizontal_reinforcement=self.create_horizontal_reinforcement()
        reinforcement+=vertical_reinforcement
        reinforcement+=horizontal_reinforcement
        return reinforcement
    
    def create_vertical_reinforcement(self):
        '''
        根据build_ele生成纵向钢筋
        '''
        reinforcement=[]
        lpath,mainline,scaleline=self.create_common_outer_section_path()
        #顶板顶层铺设线
        mainpoly=AllplanGeo.Polyline3D()
        for i in range(5):
            mainpoly+=mainline[4-i].StartPoint
        mainpoly=self.polyline3d_reverse_longthen(mainpoly,1)
        #顶板底层铺设线
        point=mainline[5].StartPoint
        maintopbottompoly=self.get_polyline_from_point(point,1)
        #底板底层铺设线
        barbottom=mainline[8]
        barbottom.Reverse()
        barbottom=self.line3d_to_polyline3d(barbottom)
        #钢筋距离偏移列表
        distancelist=self.bar_total_distance_list()
        topdistancelist=self.bar_top_distance_list()
        bottomdistancelist=self.bar_bottom_distance_list()
        heightdistancelist=self.bar_height_distance_list()
        angle_global_to_local     = AllplanGeo.Angle()
        angle_global_to_local.Deg = -90
        hooklength=((2765-155-2001-355)/2.0)
        starthookangle=135
        endhookangle=135
        r=4
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        self.shape_mat = AllplanGeo.Matrix3D()
        self.shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        #顶板顶层钢筋N21系列 N18
        #N21 13根沿顶板纵向排列 第14根位于泄水孔 由N21-7 N21-8 N21-9组合填充 第15-52根为N21 第53根为梁体中心，有泄水孔，由N21-4 N21-5 N21-6组合拼接
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,-55))
        N21=self.distancelist_barplacement_on_polyline(39,shape,distancelist,0,13,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21,True,0)
        N21=self.distancelist_barplacement_on_polyline(39,shape,distancelist,14,51-14+1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21,True,0)
        #底板底层
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,65))
        N21=self.distancelist_barplacement_on_polyline(39,shape,bottomdistancelist,8,19,barbottom,1)
        reinforcement+=self.copy_barplacement(N21,True,0)
        #翼板下侧铺设线
        barwing=AllplanGeo.Polyline3D()
        barwing+=mainline[5].StartPoint
        barwing+=mainline[6].StartPoint
        barwing+=mainline[6].EndPoint
        barwing=self.polyline3d_offset(barwing,-50,type=0)
        index,point=self.get_index_and_z_from_x_on_polyline(self.halfwidth,barwing,type=1)
        barwing.SetStartPoint(point)
        
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,0))
        N21=self.distancelist_barplacement_on_polyline(39,shape,distancelist,0,13,barwing,1)
        reinforcement+=self.copy_barplacement(N21,True,0)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,0))
        N21=self.distancelist_barplacement_on_polyline(39,shape,distancelist,14,7,barwing,1)
        reinforcement+=self.copy_barplacement(N21,True,0)
        #腹板N21铺设线
        line,point=self.get_middle_of_line_intersected_in_lines(mainline[7],mainline[0],mainline[8])
        line=AllplanGeo.Move(line,AllplanGeo.Vector3D(0,-40,0))
        line.Reverse()
        barn27sterna=self.line3d_to_polyline3d(line)
        self.barn27sterna=barn27sterna
        N21=self.distancelist_barplacement_on_polyline(39,shape,heightdistancelist,1,15,barn27sterna,2)
        reinforcement+=self.copy_barplacement(N21,True,0)
        #N21-4 N21-5 N21-6
        #N21-4
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_1_steel(base=4465,down=180,length=4835,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(4465+50,0,-55))
        N21_4=self.distancelist_barplacement_on_polyline(43,shape,distancelist,52,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_4,True,1)
        #N21-5
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=6500,down=180,length=7050,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(4650+6500/2+29,-6,-55))
        N21_5=self.distancelist_barplacement_on_polyline(44,shape,distancelist,52,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_5,True,1)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,90))
        shape.Move(AllplanGeo.Vector3D(4650+6500/2+29,6,-55-180))
        N21_5=self.distancelist_barplacement_on_polyline(44,shape,distancelist,52,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_5,True,1)
        #N21-6
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=9800,down=180,length=10350,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,-6,-55))
        N21_6=self.distancelist_barplacement_on_polyline(45,shape,distancelist,52,1,mainpoly,1)
        reinforcement+=N21_6
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,6,-55-180))
        N21_6=self.distancelist_barplacement_on_polyline(45,shape,distancelist,52,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_6,True,1)
        
        #N21-7 N21-8 N21-9
        #N21-7
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_1_steel(base=6170,down=180,length=6540,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(6170+50,-6,-55))
        N21_7=self.distancelist_barplacement_on_polyline(46,shape,distancelist,13,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_7,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        #自身y轴
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(6170+50,6,0))
        N21_7=self.distancelist_barplacement_on_polyline(46,shape,distancelist,13,1,barwing,1)
        reinforcement+=self.copy_barplacement(N21_7,True,2)
        #N21-8
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=6530,down=180,length=7080,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(6220+6530/2+200,-6,-55))
        N21_8=self.distancelist_barplacement_on_polyline(47,shape,distancelist,13,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_8,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,90))
        shape.Move(AllplanGeo.Vector3D(6220+6530/2+200,6,0))
        N21_8=self.distancelist_barplacement_on_polyline(47,shape,distancelist,13,1,barwing,1)
        reinforcement+=self.copy_barplacement(N21_8,True,2)
        #N21-9
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=6340,down=180,length=6890,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,-6,-55))
        N21_9=self.distancelist_barplacement_on_polyline(48,shape,distancelist,13,1,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_9,True,0)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,6,0))
        N21_9=self.distancelist_barplacement_on_polyline(48,shape,distancelist,13,1,barwing,1)
        reinforcement+=self.copy_barplacement(N21_9,True,0)
        #N21-3
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=4000,length=4190,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(2050,-12,-55))
        N21_3=self.distancelist_barplacement_on_polyline(42,shape,distancelist,0,18,mainpoly,1)
        reinforcement+=self.copy_barplacement(N21_3,True,2)
        #N18
        profile,hooklength=Createsteelshape.shape_N16_steel(xlength=1410,width=455,length=4000,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(55,14,-55))
        N18=self.distancelist_barplacement_on_polyline(36,shape,distancelist,31,11,mainpoly,1,2)
        reinforcement+=self.copy_barplacement(N18,True,2)
        #顶板底层 N21 20根 N23 N24 N25 N27 N27-1
        #N27 20,22,30,32-51
        #N27铺设线
        barn27=AllplanGeo.Polyline3D()
        barn27+=self.uplist[1][0]
        barn27+=self.uplist[1][1]
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N27_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(0,0,35))
        N27=self.distancelist_barplacement_on_polyline(58,shape,topdistancelist,28,2,barn27,1,3)
        reinforcement+=self.copy_barplacement(N27,True,2)
        N27=self.distancelist_barplacement_on_polyline(58,shape,topdistancelist,22,1,barn27,1)
        reinforcement+=self.copy_barplacement(N27,True,2)
        N27=self.distancelist_barplacement_on_polyline(58,shape,topdistancelist,1,20,barn27,1)
        reinforcement+=self.copy_barplacement(N27,True,2)
        #N23 24,26,28
        #N23铺设线
        barn23=AllplanGeo.Polyline3D()
        barn23+=self.uplist[4][1]
        barn23+=self.uplist[4][2]
        barn23=self.polyline3d_offset(barn23,35,type=0)
        index,point=self.get_index_and_z_from_x_on_polyline(0,barn23,1)
        newbar23=AllplanGeo.Polyline3D()
        newbar23+=point
        newbar23+=barn23[1]
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=27820,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,6))
        N23=self.distancelist_barplacement_on_polyline(54,shape,topdistancelist,24,3,newbar23,1,2)
        reinforcement+=self.copy_barplacement(N23,True,0)
        #N24 30,32
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=25820,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,6))
        N24=self.distancelist_barplacement_on_polyline(55,shape,topdistancelist,20,2,newbar23,1,2)
        reinforcement+=self.copy_barplacement(N24,True,0)
        #N25 第34根开始 隔2根 ，共9根
        barn25=AllplanGeo.Polyline3D()
        barn25+=self.uplist[4][0]
        barn25+=self.uplist[4][1]
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=24600,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(0,0,35+6))
        N25=self.distancelist_barplacement_on_polyline(56,shape,topdistancelist,2,9,barn25,1,2)
        reinforcement+=self.copy_barplacement(N25,True,0)
        N25=self.distancelist_barplacement_on_polyline(56,shape,topdistancelist,21,1,barn25,1)
        reinforcement+=self.copy_barplacement(N25,True,0)
        N25=self.distancelist_barplacement_on_polyline(56,shape,topdistancelist,29,2,barn25,1,4)
        reinforcement+=self.copy_barplacement(N25,True,0)
        #底板顶层N25铺设线
        barn25down=AllplanGeo.Polyline3D()
        barn25down+=self.downlist[4][0]
        barn25down+=self.downlist[4][1]
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=24600,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(0,0,-35-6))
        N25=self.distancelist_barplacement_on_polyline(56,shape,bottomdistancelist,0,1,barn25down,1)
        reinforcement+=N25
        N25=self.distancelist_barplacement_on_polyline(56,shape,bottomdistancelist,2,10,barn25down,1,2)
        reinforcement+=self.copy_barplacement(N25,True,0)
        N25=self.distancelist_barplacement_on_polyline(56,shape,bottomdistancelist,23,1,barn25down,1)
        reinforcement+=self.copy_barplacement(N25,True,0)
        #N27-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N27_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(0,0,35))
        N27_1=self.distancelist_barplacement_on_polyline(59,shape,topdistancelist,0,1,barn27,1)
        reinforcement+=self.copy_barplacement(N27_1,True,1)

        #N22
        barn22=AllplanGeo.Polyline3D()
        barn22+=AllplanGeo.Point3D(0,600,0)
        barn22+=AllplanGeo.Point3D()
        barn22+=AllplanGeo.Point3D(0,-600,0)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=32030,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,65))
        N22=self.div_barplacement_on_polyline(49,shape,[0],[100],[13],barn22,1)
        reinforcement+=N22
        #N22-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_1_steel(base=4205,down=180,length=4575,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(4205+50,0,65))
        N22_1=self.div_barplacement_on_polyline(50,shape,[100],[100],[1],barn22,1)
        reinforcement+=self.copy_barplacement(N22_1,True,2)
        #N22-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=2500,down=180,length=3050,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(4205+50+2500/2+258,0,65))
        N22_2=self.div_barplacement_on_polyline(51,shape,[100],[100],[1],barn22,1)
        reinforcement+=self.copy_barplacement(N22_2,True,2)
        #N22-3
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=3800,down=180,length=4350,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(4205+50+2500+258+3800/2+258,0,65))
        N22_3=self.div_barplacement_on_polyline(52,shape,[100],[100],[1],barn22,1)
        reinforcement+=self.copy_barplacement(N22_3,True,2)
        shape.Move(AllplanGeo.Vector3D(3800+258,0,0))
        N22_3=self.div_barplacement_on_polyline(52,shape,[100],[100],[1],barn22,1)
        reinforcement+=self.copy_barplacement(N22_3,True,2)
        #N22-4
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=1825,down=180,length=2375,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,65))
        N22_4=self.div_barplacement_on_polyline(53,shape,[100],[100],[1],barn22,1)
        reinforcement+=self.copy_barplacement(N22_4,True,0)

        #N26
        line26=AllplanGeo.Line3D(self.downlist[4][1],self.midlist[4][0])
        line26=self.line3d_to_polyline3d(line26)
        bar26=self.polyline3d_offset(line26,-50,x=self.halflength,type=0)
        index,point=self.get_index_and_z_from_x_on_polyline((bar26[1].Y-bar26[0].Y)/3+bar26[0].Y,bar26,1)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=25920,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(0,0,0))
        N26=self.barplacement_on_polyline(57,shape,(bar26[1].Y-bar26[0].Y)/3,(bar26[1].Y-bar26[0].Y)/3,2,bar26,1)
        reinforcement+=self.copy_barplacement(N26,True,0)
        #腹板N21-1铺设线
        line21=AllplanGeo.Line3D(self.midlist[4][1],self.midlist[4][0])
        line21=AllplanGeo.Move(line21,AllplanGeo.Vector3D(-self.halflength,0,0))
        line,point=self.get_middle_of_line_intersected_in_lines(line21,mainline[0],mainline[8])
        line=AllplanGeo.Move(line,AllplanGeo.Vector3D(0,40,0))
        line.Reverse()
        barn21sterna=self.line3d_to_polyline3d(line)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N21_steel(length=26140,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.halflength,0,0))
        N21_1=self.distancelist_barplacement_on_polyline(40,shape,heightdistancelist,3,14,barn21sterna,2)
        reinforcement+=self.copy_barplacement(N21_1,True,0)
        #N21-2 配合N21-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1810,length=2000,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(0,180-angle,90))
        shape.Move(AllplanGeo.Vector3D(self.dis02-200,0,0))
        N21_2=self.distancelist_barplacement_on_polyline(41,shape,heightdistancelist,3,14,barn21sterna,2)
        reinforcement+=self.copy_barplacement(N21_2,True,2)
        #N28
        barn28=AllplanGeo.Polyline3D()
        barn28+=AllplanGeo.Point3D(0,600,self.id02-50)
        barn28+=AllplanGeo.Point3D(0,-600,self.id02-50)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N27_steel(base=1215,x=3300,y=462,z=3332,length=4737,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.dis01,0,0))
        N28=self.div_barplacement_on_polyline(60,shape,[0],[200],[7],barn28,1)
        reinforcement+=self.copy_barplacement(N28,True,1)
        #N29
        barn29=AllplanGeo.Polyline3D()
        barn29+=AllplanGeo.Point3D(0,0,self.id02-50)
        barn29+=AllplanGeo.Point3D(0,600,self.id02-50)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N27_steel(base=1465,x=3300,y=462,z=3332,length=4987,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(self.dis01,0,0))
        N29=self.distancelist_barplacement_on_polyline(61,shape,bottomdistancelist,8,7,barn29,1,2)
        reinforcement+=self.copy_barplacement(N29,True,2)
        #腹板N30铺设线
        line30=AllplanGeo.Line3D(self.midlist[1][1],self.midlist[1][0])
        line30=AllplanGeo.Move(line30,AllplanGeo.Vector3D(-self.dis01,0,0))
        line,point=self.get_middle_of_line_intersected_in_lines(line30,mainline[0],mainline[8])
        line=AllplanGeo.Move(line,AllplanGeo.Vector3D(0,40,0))
        line.Reverse()
        barn30sterna=self.line3d_to_polyline3d(line)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N27_steel(base=1465,x=3300,y=680,z=3369,length=5024,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(0,-angle,90))
        shape.Move(AllplanGeo.Vector3D(self.dis01,0,0))
        N30=self.distancelist_barplacement_on_polyline(62,shape,heightdistancelist,3,13,barn30sterna,2)
        reinforcement+=self.copy_barplacement(N30,True,2)
        #腹板N40铺设线
        line40=AllplanGeo.Line3D(self.midlist[2][1],self.midlist[2][0])
        line40=AllplanGeo.Move(line40,AllplanGeo.Vector3D(-self.dis02,0,0))
        line,point=self.get_middle_of_line_intersected_in_lines(line40,mainline[0],mainline[8])
        line=AllplanGeo.Move(line,AllplanGeo.Vector3D(0,40,0))
        line.Reverse()
        barn40sterna=self.line3d_to_polyline3d(line)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N40_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(0,-angle,90))
        shape.Move(AllplanGeo.Vector3D(self.dis03,0,0))
        N40=self.distancelist_barplacement_on_polyline(62,shape,heightdistancelist,4,13,barn40sterna,2)
        reinforcement+=self.copy_barplacement(N40,True,2)
        #N43 目测配合N27
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=2050,length=2240,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(2050/2+50-self.dis01,0,30+self.id11-self.id21))
        N43=self.distancelist_barplacement_on_polyline(81,shape,topdistancelist,1,12,barn27,1,2)
        reinforcement+=self.copy_barplacement(N43,True,2)
        N43=self.distancelist_barplacement_on_polyline(81,shape,topdistancelist,32,1,barn27,1)
        reinforcement+=self.copy_barplacement(N43,True,2)
        #N39
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=2120,length=2310,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(0,180-angle,90))
        shape.Move(AllplanGeo.Vector3D(2120/2+50,0,0))
        N39=self.distancelist_barplacement_on_polyline(77,shape,heightdistancelist,6,6,barn40sterna,2,2)
        reinforcement+=self.copy_barplacement(N39,True,2)
        #N16-1
        profile,hooklength=Createsteelshape.shape_N16_steel(xlength=1160,width=561,length=3712,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(50+250,0,80+561))
        N16_1=self.div_barplacement_on_polyline(33,shape,[100,-300],[-200,400],[2,1],barn22,1)
        reinforcement+=self.copy_barplacement(N16_1,True,2)
        #N17-1
        profile,hooklength=Createsteelshape.shape_N16_steel(xlength=1410,width=561,length=4212,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(50,0,80+561))
        N17_1=self.div_barplacement_on_polyline(35,shape,[1000],[-400],[2],barn22,1)
        reinforcement+=self.copy_barplacement(N17_1,True,2)
        #N16
        profile,hooklength=Createsteelshape.shape_N16_steel(xlength=1160,width=561,length=3712,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(80+self.dis01,0,80+561))
        N16=self.div_barplacement_on_polyline(32,shape,[-300],[-200],[2],barn22,1)
        reinforcement+=self.copy_barplacement(N16,True,2)
        #N17
        profile,hooklength=Createsteelshape.shape_N16_steel(xlength=1140,width=561,length=3672,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(80+self.dis01,0,80+561))
        N17=self.distancelist_barplacement_on_polyline(34,shape,bottomdistancelist,9,5,barbottom,1,2)
        reinforcement+=self.copy_barplacement(N17,True,2)
        #N41
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1600,length=1790,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(1600/2+60+250,0,660))
        N41=self.div_barplacement_on_polyline(33,shape,[0],[-200],[7],barn22,1)
        reinforcement+=self.copy_barplacement(N41,True,1)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(1600/2+60+250,0,160))
        N41=self.div_barplacement_on_polyline(33,shape,[0],[-200],[7],barn22,1)
        reinforcement+=self.copy_barplacement(N41,True,1)
        #N42
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1850,length=2040,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,90))
        shape.Move(AllplanGeo.Vector3D(1850/2+60,0,660))
        N42=self.distancelist_barplacement_on_polyline(80,shape,bottomdistancelist,8,6,barbottom,1,2)
        reinforcement+=self.copy_barplacement(N42,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,0,90))
        shape.Move(AllplanGeo.Vector3D(1850/2+60,0,160))
        N42=self.distancelist_barplacement_on_polyline(80,shape,bottomdistancelist,8,6,barbottom,1,2)
        reinforcement+=self.copy_barplacement(N42,True,2)
        reinforcement=[]
        return reinforcement
    
    def create_horizontal_reinforcement(self):
        '''
        根据build_ele生成横向钢筋
        '''
        matxz=AllplanGeo.Matrix3D()
        matxz.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        outsect,outmain,outscale=self.create_common_outer_section_path()
        start_point=outmain[3].GetEndPoint()
        end_point=start_point+AllplanGeo.Vector3D(2*self.halflength,0,0)
        angle_global_to_local     = AllplanGeo.Angle()
        angle_global_to_local.Deg = -90
        reinforcement=[]
        hooklength=((2765-155-2001-355)/2.0)
        starthookangle=135
        endhookangle=135
        r=4
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        #N1 N5 N7 N8为顶板顶层横向钢筋
        #N1系 7根N1 1根N1-2 9根N1 1根N1-2 7根N1 起始偏移50 钢筋间距100 1根N1-2两边各配1根N1-1 关于梁跨中心镜像 铺设线：顶板顶层梁体中心线
        #总计 （7+9+7）*2=46根N1 （1+1）*2=4根N1-2 4*2=8根N1-1
        #N1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,0))
        start_point=outmain[0].GetStartPoint()
        dislistn1=[50,200,200]
        barcountlistn1=[7,9,7]
        N1=self.div_barplacement_on_polyline(1,shape,dislistn1,[100,100,100],barcountlistn1,self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N1,True,1)
        #N1-2 N1中混杂的两根
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,0))
        N1_2=self.div_barplacement_on_polyline(3,shape,[750],[1000],[2],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N1_2,True,1)
        #N1-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,7600/2-1895,0))
        N1_1=self.get_combine_barplacement(N1_2,2,shape)
        reinforcement+=self.copy_barplacement(N1_1,True,2)
        
        #N5
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_steel(diameter=18)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,0))
        dislistn5=[self.halflength-94*125-20*100,100+125*2,125*2,125*2,125*2,125*2,125*2,125*2,100+125*2]
        barcountlistn5=[20,93-80-1,80-40-1,40-26-1,26*2-1,40-26-1,80-40-1,93-80-1,20]
        bardislistn5=[100,125,125,125,125,125,125,125,100]
        N5=self.div_barplacement_on_polyline(10,shape,dislistn5,bardislistn5,barcountlistn5,self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N5,True,1)

        #N5-2 N5中混杂的4根
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N5_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,0))

        start_point=start_point+AllplanGeo.Vector3D(self.halflength-80*125,0,0)
        N5_2=self.div_barplacement_on_polyline(12,shape,[self.halflength-80*125],[125*(80-26)],[2],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N5_2,True,1)

        #N5-1
        
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N5_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,-30-16/2,-50))
        start_point=outmain[4].GetStartPoint()
        barn5_1=self.get_polyline_from_point(start_point)
        dislistn5_1=[50+22/2+18/2,100,125*2,125*2,125*2,125*2,100-22/2-18/2-22/2]
        barcountlistn5_1=[45,94-80,80-26-1,26*2-1,80-26-1,94-80,45]
        bardislistn5_1=[100,125,125,125,125,125,100]
        N5_1=self.div_barplacement_on_polyline(11,shape,dislistn5_1,bardislistn5_1,barcountlistn5_1,barn5_1)
        reinforcement+=self.copy_barplacement(N5_1,True,0)
        #N5-3  配套N5-2
        
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N5_3_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,-30-18/2,-35-18/2))
        start_point=start_point+AllplanGeo.Vector3D(self.halflength-80*125,0,0)
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N5_3=self.div_barplacement_on_polyline(13,shape,[self.halflength-80*125],[125*(80-26)],[2],barn5_1)
        reinforcement+=self.copy_barplacement(N5_3,True,2)
        #N5-4 配合N5和N5-2，顺接N1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_1_steel(base=3700,x=900,y=27,z=901,down=180,length=5067,diameter=18)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,7600/2-3700,-35-18/2))
        dislistn5_4=[4550,(93-40)*125]
        barcountlistn5_4=[2,1]
        bardislistn5_4=[125,125]
        N5_4=self.div_barplacement_on_polyline(14,shape,dislistn5_4,bardislistn5_4,barcountlistn5_4,self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N5_4,True,2)
        #N7
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N7_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,7600/2-3700,-35-18/2))
        dislistn7=[self.halflength-18-23*4*125,2*125]
        barcountlistn7=[1,23]
        bardislistn7=[125*2,125*4]
        N7=self.div_barplacement_on_polyline(18,shape,dislistn7,bardislistn7,barcountlistn7,self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N7,True,1)
        
        #N8
        profiles,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N7_steel(a=498,b=705,c=3170)
        profilee,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N7_steel(a=212,b=300,c=3742)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profiles,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(1650-20,0,-35-18/2+self.height))
        endshape = ProfileShapeBuilder.create_profile_shape(profilee,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        endshape.Move(AllplanGeo.Vector3D(4450-20,0,-35-18/2+self.height))
        N8=AllplanReinf.BarPlacement(19,15,shape,endshape)
        reinforcement+=self.copy_barplacement(N8,False,1)
        #N2 N14为顶板底层横向钢筋
        #N2-2 2根 N2 4根 （N1-2配N2-1两根）1组 N210根 （N1-2配N2-1两根）1组 N2 7根
        #N2-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,22/2))
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N2_2=self.div_barplacement_on_polyline(6,shape,[50],[100],[2],self.bartopmidlist)
        reinforcement+=self.copy_barplacement(N2_2,True,1)
        #N2
        barvec=self.line3d_to_vector3d(self.bartopmidlist.GetLine(1))
        barvec0=self.line3d_to_vector3d(self.bartopmidlist.GetLine(0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,22/2))
        start_point=self.bartopmidlist[0]
        line=self.bartopmidlist
        N2=self.div_barplacement_on_polyline(22,shape,[250,200,200],[100,100,100],[5,9,7],line)
        reinforcement+=self.copy_barplacement(N2,True,1)
        #N1-2 N2-1
        #N1-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,0))
        shape.Move(AllplanGeo.Vector3D(0,0,22/2))
        start_point=self.bartopmidlist[0]
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N1_21=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    3, shape,
                    start_point,
                    position_point,
                    750-22/2,100,1)
        reinforcement.append(N1_21)
        N1_21.Move(AllplanGeo.Vector3D(-22,0,0))
        N1_21c=self.reflection_barplacement(N1_21,True,1)
        reinforcement.append(N1_21c)
        start_point=self.bartopmidlist[0]+AllplanGeo.Vector3D(1750-22,0,(1750-22-barvec0.GetLength())*barvec.Z/barvec.X)
        dislistn1_21=[0]
        barcountlistn1_21=[1]
        bardislistn1_21=[100/barvec.X*barvec.GetLength()]
        for i in range(0,len(dislistn1_21)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn1_21[i],0,dislistn1_21[i]*barvec.Z/barvec.X)
            position_point=start_point+AllplanGeo.Vector3D(100,0,100*barvec.Z/barvec.X)
            N1_22=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    3, shape,
                    start_point,
                    position_point,
                    -22/2, bardislistn1_21[i], barcountlistn1_21[i])
            reinforcement.append(N1_22)
            N1_22c=self.reflection_barplacement(N1_22,True,1)
            reinforcement.append(N1_22c)
        #N2-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,0))
        shape.Move(AllplanGeo.Vector3D(0,7600/2-1895,22/2))
        N2_1=self.get_combine_barplacement(N1_21,5,shape)
        reinforcement+=self.copy_barplacement(N2_1,True,2)
        N2_2=self.get_combine_barplacement(N1_22,5,shape)
        reinforcement+=self.copy_barplacement(N2_2,True,2)
        '''N14系 共4排 
        第一排 起始偏移50 钢筋间距100 1根N14-1 2根N14 铺设线：顶板顶层中心线向下偏移某值；
        第二排 起始偏移50 钢筋间距100 3根N14 铺设线：顶板顶层中心线向下偏移某值；
        第三排 起始偏移50 钢筋间距100 1根N14-2 2根N14 铺设线:顶板顶层中心线向下偏移某值；
        第四排 起始偏移50 钢筋间距100 20根N14 铺设线:顶板顶层中心线向下偏移某值；
        第五排 起始偏移2550 钢筋间距100 20根N14 隔100 钢筋间距125 2根N14-3 52根N14 1根N14-3 40根N14 铺设线:顶板底层中心线向上偏移某值；
        以上均关于梁跨中心对称铺设
        N14-3还关于梁体中心对称铺设
        根数总计：（2+3+2+20）*2+(20+52+40)*2-1=277根N14 2根N14-1 2根N14-2 (2+1)*2*2=12根N14-3
        '''
        #N14-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=10460,length=10714,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(-19,0,-116-8))
        line=self.bartoptoplist
        N14_1=self.div_barplacement_on_polyline(28,shape,[50],[100],[1],line)
        reinforcement+=self.copy_barplacement(N14_1,True,1)
        #N14-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=9560,length=9814,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(-19,0,-240-8))
        line=self.bartoptoplist
        N14_2=self.div_barplacement_on_polyline(29,shape,[50],[100],[1],line)
        reinforcement+=self.copy_barplacement(N14_2,True,1)
        #N14
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=6860,length=7114,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(-19,0,-116-8))
        N14=self.div_barplacement_on_polyline(27,shape,[150],[100],[2],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N14,True,1)
        shape.Move(AllplanGeo.Vector3D(0,0,-62))
        N14=self.div_barplacement_on_polyline(27,shape,[50],[100],[3],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N14,True,1)
        shape.Move(AllplanGeo.Vector3D(0,0,-62))
        N14=self.div_barplacement_on_polyline(27,shape,[150],[100],[2],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N14,True,1)
        shape.Move(AllplanGeo.Vector3D(0,0,-62))
        N14=self.div_barplacement_on_polyline(27,shape,[50],[100],[20],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N14,True,1)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,8))
        N14=self.div_barplacement_on_polyline(27,shape,[2550,100+2*125,2*125,2*125,100+2*125],[100,125,125,125,100],[20,52,79,52,20],self.bartopmidlist)
        reinforcement+=N14
        #N14-3
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N14_3_steel()
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,180,180))
        shape.Move(AllplanGeo.Vector3D(0,6860/2-3310,8))
        N14_3=self.div_barplacement_on_polyline(29,shape,[4550,53*125],[125,125],[2,1],self.bartopmidlist)
        reinforcement+=self.copy_barplacement(N14_3,True,2)
        #N44 N44-2间距同N1系
        #N44-2 倒角钢筋 
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1100,length=1448)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-self.angle1list[0],0,0))
        N44_2=self.div_barplacement_on_polyline(84,shape,[50],[100],[25],self.barinupcharmlist)
        reinforcement+=self.copy_barplacement(N44_2,True,2)
        
        #N44 倒角钢筋
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=600,length=790,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-self.angle1list[2],0,0))
        N44=self.div_barplacement_on_polyline(82,shape,[2550,125,125],[100,125,100],[21,94*2-1,21],self.barinupcharmlist)
        reinforcement+=self.copy_barplacement(N44,True,0)
        #N31 翼板
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1200,length=1454,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q40/self.c40)
        shape.Rotate(RotationAngles(angle*180/math.pi,0,0))
        shape.Move(self.vector_on_slope(angle,-(1200/2+60),35+16/2))
        wing=outmain[5].GetStartPoint()
        wingpoly=self.get_polyline_from_point(wing)
        N31=self.div_barplacement_on_polyline(63,shape,[50,125,125],[100,125,100],[46,94*2-1,46],wingpoly)
        reinforcement+=self.copy_barplacement(N31,True,0)
        '''
        N32 N33系
        N32 起始偏移50 间距100 24根 偏移100 顺接N33系 铺设同时关于梁跨中心和梁体中心对称
        N33系 N33 起始偏移2450 间距100 22根 偏移125  间距125 13根N33 1组N33-1和N33-2 53根N33 1组N33-1和N33-2 26根N33 铺设同时关于梁跨中心和梁体中心对称
        总计： N33：（（22+13+53+26）*2-1）*2=454根（图纸为462根） N33-1:8根 N33-2:8根 N32:24*4=96根
        '''
        #N32
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=2850,length=3040,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q30/self.c30)
        shape.Rotate(RotationAngles(angle*180/math.pi,0,0))
        shape.Move(self.vector_on_slope(angle,-(2850/2-300),65))
        wing=outmain[6].GetStartPoint()
        wingpoly=self.get_polyline_from_point(wing)
        N32=self.div_barplacement_on_polyline(64,shape,[50],[100],[24],wingpoly)
        reinforcement+=self.copy_barplacement(N32,True,2)
        #N33
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N33_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        N33=self.div_barplacement_on_polyline(65,shape,[2450-self.dis03,125,2*125,2*125,2*125,2*125,125],[100,125,125,125,125,125,100],[22,13,53,51,53,13,22],self.n33barline)
        reinforcement+=self.copy_barplacement(N33,True,0)
        #N33-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N33_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        N33_1=self.div_barplacement_on_polyline(66,shape,[self.halflength-self.dis03-80*125,(80-26)*125],[125,125],[1,1],self.n33barline)
        reinforcement+=self.copy_barplacement(N33_1,True,2)
        #N33-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N33_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q30/self.c30)
        shape.Move(self.vector_on_slope(angle,1565,0))
        N33_2=self.div_barplacement_on_polyline(67,shape,[self.halflength-self.dis03-80*125,(80-26)*125],[125,125],[1,1],self.n33barline)
        reinforcement+=self.copy_barplacement(N33_2,True,2)
        #N44系铺设线
        n44p,anglen44=self.get_point_and_angle_from_offset_arc3d(outscale[9],35+20/2)
        n44bar=self.get_polyline_from_point(n44p)
        #N44-3
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=900,length=1216,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-anglen44,0,0))
        N44_3=self.div_barplacement_on_polyline(85,shape,[50],[100],[1],n44bar)
        reinforcement+=self.copy_barplacement(N44_3,True,2)
        #N44-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=900,length=1090,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-anglen44,0,0))
        N44_1=self.div_barplacement_on_polyline(83,shape,[150,125,100],[100,125,100],[45,187,45],n44bar)
        reinforcement+=self.copy_barplacement(N44_1,True,0) 
        '''
        N12系 同N5系 N12--N5  1根N12-1配2根N12-2 取代其余N5系
        '''
        #N12
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N6_steel(a=338,b=5032,c=2841,d=2757,e=689,f=180,r=255,length=11750,diameter=18)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        N12=self.div_barplacement_on_polyline(24,shape,dislistn5,bardislistn5,barcountlistn5,self.barbottombottomlist)
        reinforcement+=N12
        #N12-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=1185,down=180,length=1831,diameter=18)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N12_2=self.div_barplacement_on_polyline(26,shape,[4550,13*125,5000,1750],[125,125,125,125],[2,1,1,1],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N12_2,True,1)
        #N12-1 搭配N12-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N12_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,-1,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,5032/2-1700,0))
        N12_1=self.get_combine_barplacement(N12_2,25,shape)
        reinforcement+=self.copy_barplacement(N12_1,True,2)
        '''
        N6系
        N6-1 起始偏移350 间距100 12根 偏移100 顺接N6-2 间距100 6根 偏移100 顺接N6 间距100 4根 铺设关于梁跨中心对称
        总计： N6：4*2=8根 N6-1:12*2=24根 N6-2:6*2=12根 (图纸为28根)
        '''
        #N6系
        #N6
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N6_steel()
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shapen6 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,-1,starthookangle,endhookangle)
        N6=self.div_barplacement_on_polyline(15,shapen6,[2150],[100],[4],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N6,True,1)
        #N6-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N6_steel(a=72,b=5344,c=2996,d=2906,e=727,f=200,r=54,length=11880,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shapen6_1 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,-1,starthookangle,endhookangle)
        N6_1=self.div_barplacement_on_polyline(16,shapen6_1,[350+8],[100],[8],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N6_1,True,1)
        N6_1=self.div_barplacement_on_polyline(16,shapen6_1,[350-8],[200],[4],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N6_1,True,1)
        #N6-2
        shapen6_1.Move(AllplanGeo.Vector3D(1150+8,0,50))
        shapen6.Move(AllplanGeo.Vector3D(2050+8,0,50))
        N6_2=AllplanReinf.BarPlacement(17,10,shapen6_1,shapen6)
        reinforcement+=self.copy_barplacement(N6_2,False,1)
        shapen6_1.Move(AllplanGeo.Vector3D(-16,0,0))
        shapen6.Move(AllplanGeo.Vector3D(-316,0,0))
        N6_2=AllplanReinf.BarPlacement(17,4,shapen6_1,shapen6)
        reinforcement+=self.copy_barplacement(N6_2,False,1)
        '''
        N10 N15沿腹板方向铺设
        N10 起始偏移50，间隔100 42根 铺设同时关于梁跨中心和梁体中心对称
        N10-1 起始偏移71，间隔100 26根 铺设同时关于梁跨中心和梁体中心对称
        '''
        #N10 #N15系
        #N15
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=3060,length=3314,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(angle,0,0))
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[50],[100],[1],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[3],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-160,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[3],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[16],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[50],[100],[4],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[3],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-160,0))
        N15=self.div_barplacement_on_polyline(31,shape,[750],[100],[16],self.barn15)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(angle,0,0))
        N15=self.div_barplacement_on_polyline(31,shape,[4550],[125],[95*2-1],self.barn10)
        reinforcement+=self.copy_barplacement(N15,True,0)
        #N15的一条特殊铺设线
        barn15sp=AllplanGeo.Polyline3D()
        y=(self.barn15)[0].Y
        z=(self.barn15)[0].Z
        point0=AllplanGeo.Point3D(850,y-720,z)
        point1=AllplanGeo.Point3D(2050,y-640,z)
        bar10part=self.barn10.GetLine(3)
        bar10part=self.line3d_to_polyline3d(bar10part)
        index,point2=self.get_index_and_z_from_x_on_polyline(3650,bar10part)
        barn15sp+=point0
        barn15sp+=point1
        barn15sp+=point2
        barn15sp+=(self.barn10)[3]
        N15=self.div_barplacement_on_polyline(31,shape,[-600,200,100],[100,200,100],[6,5,24],barn15sp)
        reinforcement+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[-800],[100],[2],barn15sp)
        reinforcement+=self.copy_barplacement(N15,True,2)
        #N10
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=3060,length=3376,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(angle,0,0))
        N10=self.div_barplacement_on_polyline(22,shape,[350],[100],[42],self.barn10)
        reinforcement+=self.copy_barplacement(N10,True,2)
        #N10-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=2490,length=2838,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(angle,0,0))
        N10_1=self.div_barplacement_on_polyline(23,shape,[71],[100],[23],self.barn10)
        reinforcement+=self.copy_barplacement(N10_1,True,2)
        '''
        底板顶层N4系N3系N37系N35系
        N4系：起始偏移50 N4 间距100 3根 铺设同时关于梁跨中心和梁体中心对称
        N3系：起始偏移350 N3-1 1根 偏移直径 接N3 间距100 13根 铺设关于梁跨中心对称
        N37系：起始偏移1650 N37 间距100 29根 前9根两根一组 铺设关于梁跨中心对称
        N35系：起始偏移4550 间距125 N35-2 2根 N35 20根 N35-2 1根 N35 31根  N35-2 1根 N35 31根 N35-2 1根 N35 8根铺设关于梁跨中心对称 1根N35-2配2根35-1
        总计： N4:3*4=12根 N3:13*2=26根 N3-1:1*2=2根 N37:(29+9)*2=76根 N35:(20+31+31+8)*2-1=179根 N35-2:(2+1+1+1)*2=10根 N35-1:10*2=20根
        '''
        #N4
        #todo 挖槽，挖完钢筋长度修正
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=2115,length=2463,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        shape.Move(AllplanGeo.Vector3D(0,self.c10-2115/2-50,0))
        N4=self.div_barplacement_on_polyline(9,shape,[50],[100],[3],self.barbottommidlist)
        reinforcement+=self.copy_barplacement(N4,True,2)
        
        #N3-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=3000,length=3348,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N3_1=self.div_barplacement_on_polyline(9,shape,[250+22],[100],[1],self.barbottommidlist)
        reinforcement+=self.copy_barplacement(N3_1,True,1)
        shape.Rotate(RotationAngles(180,0,0))
        N3_1=self.div_barplacement_on_polyline(9,shape,[250+22],[100],[1],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N3_1,True,1)
        #N3
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5704,length=6102,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N3=self.div_barplacement_on_polyline(9,shape,[350],[100],[13],self.barbottommidlist)
        reinforcement+=self.copy_barplacement(N3,True,1)
        #N37
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551,length=5837-98,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape1 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape1.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551+17*7,length=5837-98+17*7,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape2 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,176,176,starthookangle,endhookangle)
        shape2.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551+18*7,length=5837-98+18*7,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape5 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,176,176,starthookangle,endhookangle)
        shape5.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5747,length=5837+98,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape6 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,176,176,starthookangle,endhookangle)
        shape6.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551+7*8,length=5837-98+7*8,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape3 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,176,176,starthookangle,endhookangle)
        shape3.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551,length=5837-98,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape4 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,176,176,starthookangle,endhookangle)
        shape4.Rotate(RotationAngles(180,0,0))
        index,point1=self.get_index_and_z_from_x_on_polyline(1650,self.barbottommidlist)
        shape1.Move(AllplanGeo.Vector3D(point1))
        index,point2=self.get_index_and_z_from_x_on_polyline(3350,self.barbottommidlist)
        shape2.Move(AllplanGeo.Vector3D(point2))
        N37=AllplanReinf.BarPlacement(9,18,shape1,shape2)
        reinforcement+=self.copy_barplacement(N37,False,1)
        index,point3=self.get_index_and_z_from_x_on_polyline(1650+22,self.barbottommidlist)
        shape4.Move(AllplanGeo.Vector3D(point3))
        index,point4=self.get_index_and_z_from_x_on_polyline(2450+22,self.barbottommidlist)
        shape3.Move(AllplanGeo.Vector3D(point4))
        N37=AllplanReinf.BarPlacement(9,9,shape4,shape3)
        reinforcement+=self.copy_barplacement(N37,False,1)
        
        index,point5=self.get_index_and_z_from_x_on_polyline(3450,self.barbottommidlist)
        shape5.Move(AllplanGeo.Vector3D(point5))
        index,point6=self.get_index_and_z_from_x_on_polyline(4450,self.barbottommidlist)
        shape6.Move(AllplanGeo.Vector3D(point6))
        N37=AllplanReinf.BarPlacement(9,11,shape5,shape6)
        reinforcement+=self.copy_barplacement(N37,False,1)
        #N35系：起始偏移4650 间距125 N35-2 2根 N35 20根 N35-2 1根 N35 31根  N35-2 1根 N35 31根 N35-2 1根 N35 8根铺设关于梁跨中心对称 1根N35-2配2根35-1
        #N35-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=1185,down=180,length=1799,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        N35_2=self.div_barplacement_on_polyline(74,shape,[4550,21*125,32*125,32*125],[125,125,125,125],[2,1,1,1],self.barbottommidlist)
        reinforcement+=self.copy_barplacement(N35_2,True,1)
        #N35-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N14_3_steel(base=1950,down=180,length=2384,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,1950-5524/2,0))
        N35_1=self.get_combine_barplacement(N35_2,73,shape)
        reinforcement+=self.copy_barplacement(N35_1,True,2)
        #N35
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5524,length=5778,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N35=self.div_barplacement_on_polyline(72,shape,[4800,2*125,2*125,2*125,2*125,2*125,2*125],[125,125,125,125,125,125,125],[20,31,31,15,31,31,20],self.barbottommidlist)
        reinforcement+=N35
        #N9 N6 N12系
        #N9系 起始偏移50 间距100 3根 铺设同时关于梁跨中心和梁体中心对称 1根N9配1根N9-1 总计：各12根
        #N9
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N9_steel()
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Move(AllplanGeo.Vector3D(0,5344/2-714,0))
        N9=self.div_barplacement_on_polyline(20,shape,[50],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N9,True,2)
        #N9-1
        #TODO 契合槽,钢筋长度修正
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1930,length=2268,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,5344/2-714-1930/2-20,0))
        N9_1=self.div_barplacement_on_polyline(21,shape,[50],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N9_1,True,2)
        #N34系 梁下底倒角横钢筋 
        #34 起始偏移1550 间距100 30根 偏移100 间距125 94根 铺设同时关于梁跨中心和梁体中心对称
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=800,length=990,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape34s = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape34s.Rotate(RotationAngles(-self.angle2list[1],0,0))
        N34=self.div_barplacement_on_polyline(68,shape34s,[1550],[100],[30],self.barindowncharmlist)
        reinforcement+=self.copy_barplacement(N34,True,2)
        index,point1=self.get_index_and_z_from_x_on_polyline(4550,self.barindowncharmlist)
        shape34s.Move(AllplanGeo.Vector3D(point1))
        shape34e = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape34e.Rotate(RotationAngles(180+self.angle2list[4],0,0))
        index,point2=self.get_index_and_z_from_x_on_polyline(self.halflength-125,self.barindowncharmlist)
        shape34e.Move(AllplanGeo.Vector3D(point2))
        N34=AllplanReinf.BarPlacement(68,94,shape34s,shape34e)
        reinforcement+=self.copy_barplacement(N34,False,2)
        shape34e = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape34e.Rotate(RotationAngles(180+self.angle2list[4],0,0))
        N34=self.div_barplacement_on_polyline(68,shape34e,[self.halflength],[125],[1],self.barindowncharmlist)
        reinforcement+=self.copy_barplacement(N34,True,0)
        #N34-1 起始偏移50 间距100 15根 铺设同时关于梁跨中心和梁体中心对称
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=950,length=1266,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-self.angle2list[0],0,0))
        N34_1=self.div_barplacement_on_polyline(69,shape,[50],[100],[15],self.barindowncharmlist)
        reinforcement+=self.copy_barplacement(N34_1,True,2)
        #N34-3 在N34-1前面铺一根
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=950,length=1204,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-self.angle2list[0],0,0))
        N34_3=self.div_barplacement_on_polyline(69,shape,[50-18],[100],[1],self.barindowncharmlist)
        reinforcement+=self.copy_barplacement(N34_3,True,2)
        #N19 底板中间4排 第一排高600 偏移350 间距100 1根 偏移100 15根 第二排高400 偏移550 间距100 10根 斜向下 第三排高200 偏移550 间距100 3根 第四排高100 偏移450 间距100 16根 铺设关于梁跨中心对称
        #todo 需要修正中间两行
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=(2650+600/self.q20*self.c20)*2,length=(2650+600/self.q20*self.c20)*2+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        shape.Move(AllplanGeo.Vector3D(0,0,600))
        N19=self.div_barplacement_on_polyline(37,shape,[350,200],[100,100],[1,15],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N19,True,1)
        
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=(2650+500/self.q20*self.c20)*2,length=(2650+500/self.q20*self.c20)*2+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,400))
        N19=self.div_barplacement_on_polyline(37,shape,[550],[100],[10],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N19,True,1)

        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=(2650+200/self.q20*self.c20)*2,length=(2650+200/self.q20*self.c20)*2+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,200))
        N19=self.div_barplacement_on_polyline(37,shape,[550],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N19,True,1)

        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=(2650+100/self.q20*self.c20)*2,length=(2650+100/self.q20*self.c20)*2+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,100))
        N19=self.div_barplacement_on_polyline(37,shape,[450],[100],[16],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N19,True,1)
        #N20 补在N19的前三根 底板中间4排 第一排高600 偏移50 间距100 3根第二排高400 偏移250 间距100 3根 斜向下 第三排高200 偏移250 间距100 3根 第四排高100 偏移150 间距100 3根 铺设同时关于梁跨中心和梁体中心线对称
        #todo 需要修正中间两行
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1900+600/self.q20*self.c20,length=1900+600/self.q20*self.c20+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        shape.Move(AllplanGeo.Vector3D(0,1700+600/self.q20*self.c20/2,600))
        N20=self.div_barplacement_on_polyline(38,shape,[50],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N20,True,2)
        
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1900+400/self.q20*self.c20,length=1900+400/self.q20*self.c20+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,1700+400/self.q20*self.c20/2,400))
        N20=self.div_barplacement_on_polyline(38,shape,[250],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N20,True,2)

        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1900+200/self.q20*self.c20,length=1900+200/self.q20*self.c20+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,1700+200/self.q20*self.c20/2,200))
        N20=self.div_barplacement_on_polyline(38,shape,[250],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N20,True,2)

        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=1900+100/self.q20*self.c20,length=1900+100/self.q20*self.c20+254,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,1700+100/self.q20*self.c20/2,100))
        N20=self.div_barplacement_on_polyline(38,shape,[150],[100],[3],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N20,True,2)
        #N34-2
        point1=AllplanGeo.Point3D((self.bottomholeradius+50)/math.sqrt(2),self.bottomholehalflength-self.bottomholeradius+(self.bottomholeradius+50)/math.sqrt(2),0)
        point2=AllplanGeo.Point3D((self.bottomholeradius+50)/math.sqrt(2),self.bottomholehalflength-self.bottomholeradius+(self.bottomholeradius+50)/math.sqrt(2),self.id02)
        barn34_2=AllplanGeo.Polyline3D()
        barn34_2+=point1
        barn34_2+=point2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=500,length=690,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,90,45))
        N34_2=self.div_barplacement_on_polyline(70,shape,[160,100,200,100],[100,100,100,100],[1,1,1,1],barn34_2,2)
        reinforcement+=self.copy_barplacement(N34_2,True,2)
        #N38
        point1=AllplanGeo.Point3D(250+60,600,0)
        point2=AllplanGeo.Point3D(250+60,600,self.id02)
        barn38=AllplanGeo.Polyline3D()
        barn38+=point1
        barn38+=point2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N38_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(0,90,90))
        N38=self.div_barplacement_on_polyline(76,shape,[200],[150],[3],barn38,2)
        reinforcement+=self.copy_barplacement(N38,True,2)
        #N45 翼板两排 斜45度钩筋 梅花型铺设 
        #第一排：偏移250 间距500 3根 偏移700 间距500 6根 偏移100+3*125 间距500 23根 间隔3*125 1根 铺设同时关于梁跨中心和梁体中心线对称
        #第二排：偏移450 间距500 3根 偏移700 间距500 2根 偏移600 间距400 2根 偏移600 间距400 1根   偏移3*100+125 间距500 23根 铺设同时关于梁跨中心和梁体中心线对称
        #总计（（3+6+23+3+2+2+1+23）*2+1）*2=254根 （图纸为256根）
        #第一排
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=213,length=440,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.halfwidth-145-60-20,-213/2+50))
        N45=self.div_barplacement_on_polyline(86,shape,[250,700,100+3*125],[500,500,500],[3,6,23],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N45,True,2)
        N45=self.div_barplacement_on_polyline(86,shape,[self.halflength],[500],[1],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N45,True,0)
        #第二排
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=239,length=466,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.halfwidth-145-60-150*3-20,-239/2+40))
        N45=self.div_barplacement_on_polyline(86,shape,[450,700,600,600,3*100+125],[500,500,400,400,500],[3,2,2,1,23],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N45,True,2)
        #N46 翼板两排 斜45度钩筋 梅花型铺设 共4排 第一二排参照N45第一二排 第三四排同上
        #第一排：偏移250 间距500 3根 偏移700 间距500 6根 偏移100+3*125 间距500 23根 间隔3*125 1根 铺设同时关于梁跨中心和梁体中心线对称
        #第二排：偏移450 间距500 3根 偏移700 间距500 2根 偏移600 间距400 2根 偏移600 间距400 1根   偏移3*100+125 间距500 23根 铺设同时关于梁跨中心和梁体中心线对称
        #总计（（3+6+23+3+2+2+1+23）*2+1）*2=254根 （图纸为256根）
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=275,length=501,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.halfwidth-145-60-150*6-20,-275/2+30))
        N46=self.div_barplacement_on_polyline(86,shape,[250,700,100+3*125],[500,500,500],[3,6,23],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N46,True,2)
        N46=self.div_barplacement_on_polyline(86,shape,[self.halflength],[500],[1],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N46,True,0)
        #第二排
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=350,length=576,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.halfwidth-145-60-150*9-20,-350/2+30))
        N46=self.div_barplacement_on_polyline(86,shape,[450,700,600,600,3*100+125],[500,500,400,400,500],[3,2,2,1,23],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N46,True,2)
        #第三排
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=425,length=651,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.halfwidth-145-60-150*12-20,-425/2+30))
        N46=self.div_barplacement_on_polyline(86,shape,[250,700,100+3*125],[500,500,500],[3,6,23],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N46,True,2)
        N46=self.div_barplacement_on_polyline(86,shape,[self.halflength],[500],[1],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N46,True,0)
        #第四排
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=500,length=726,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.halfwidth-145-60-150*15-20,-500/2+30))
        N46=self.div_barplacement_on_polyline(86,shape,[450,700,600,600,3*100+125],[500,500,400,400,500],[3,2,2,1,23],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N46,True,2)
        #N47
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=248,length=475,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,200,-248/2+30))
        flag=-1
        for i in range(5):
            N47=self.div_barplacement_on_polyline(86,shape,[4550+125],[500],[24],self.bartoptoplist)
            reinforcement+=self.copy_barplacement(N47,True,2)
            shape.Move(AllplanGeo.Vector3D(flag*250,400,0))
            flag=-flag
        #N48
        base=253
        length=480
        flag=1
        for i in range(6):
            profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=base,length=length,diameter=12)
            shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
            shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                               shape_mat,
                                                               shape_props,
                                                               0,hooklength,hooklength,starthookangle,endhookangle)
            shape.Rotate(RotationAngles(90,0,45))
            shape.Move(AllplanGeo.Vector3D(flag*100,200+i*400,-base/2+30))
            N48=self.div_barplacement_on_polyline(89,shape,[1850],[500],[6],self.bartoptoplist)
            reinforcement+=self.copy_barplacement(N48,True,2)
            flag=-flag
            base+=51.6
            length+=51.6
        #N49
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=238,length=465,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,200,238/2))
        flag=-1
        for i in range(5):
            N49=self.div_barplacement_on_polyline(91,shape,[4550+125],[500],[24],self.barbottombottomlist)
            reinforcement+=self.copy_barplacement(N49,True,2)
            shape.Move(AllplanGeo.Vector3D(flag*250,400,0))
            flag=-flag
        #N50
        base=259
        length=486
        flag=1
        for i in range(6):
            profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=base,length=length,diameter=12)
            shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
            shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                               shape_mat,
                                                               shape_props,
                                                               0,hooklength,hooklength,starthookangle,endhookangle)
            shape.Rotate(RotationAngles(90,0,45))
            shape.Move(AllplanGeo.Vector3D(flag*100,200+i*400,base/2))
            N50=self.div_barplacement_on_polyline(92,shape,[1850],[500],[6],self.barbottombottomlist)
            reinforcement+=self.copy_barplacement(N50,True,2)
            flag=-flag
            base+=70
            length+=70
        #N51
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=422,length=649,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        shape.Move(AllplanGeo.Vector3D(0,2700-422/2+(65+5*150)/self.q20*self.c20,65+5*150))
        for i in range(4):
            N51=self.div_barplacement_on_polyline(93,shape,[4550+125],[500],[24],self.barbottombottomlist)
            reinforcement+=self.copy_barplacement(N51,True,2)
            shape.Move(AllplanGeo.Vector3D(0,450/self.q20*self.c20,450))
        #N52
        #todo 修改铺设方式
        base=476
        length=603
        for i in range(6):
            profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=base,length=length,diameter=12)
            shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
            shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                               shape_mat,
                                                               shape_props,
                                                               0,hooklength,hooklength,starthookangle,endhookangle)
            shape.Rotate(RotationAngles(180,0,0))
            shape.Move(AllplanGeo.Vector3D(4250-i*500,-base/2+50,0))
            N52=self.div_barplacement_on_polyline(94,shape,[65+5*150],[450],[4],self.barn27sterna,2)
            reinforcement+=self.copy_barplacement(N52,True,2)
            base+=103
            length+=103
        #N53 4 3 4 3排列
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=558,length=785,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,0,-558/2+30))
        N53=self.div_barplacement_on_polyline(95,shape,[150,300],[300,400],[1,3],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N53,True,1)
        for i in range(10):
            shape.Move(AllplanGeo.Vector3D(0,self.bar_top_distance_list()[2*i+2]-self.bar_top_distance_list()[2*i],0))
            N53=self.div_barplacement_on_polyline(95,shape,[150,300],[300,400],[1,3],self.bartoptoplist)
            reinforcement+=self.copy_barplacement(N53,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,100,-558/2+30))
        N53=self.div_barplacement_on_polyline(95,shape,[250],[400],[3],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N53,True,2)
        for i in range(9):
            shape.Move(AllplanGeo.Vector3D(0,self.bar_top_distance_list()[2*i+3]-self.bar_top_distance_list()[2*i+1],0))
            N53=self.div_barplacement_on_polyline(95,shape,[250],[400],[3],self.bartoptoplist)
            reinforcement+=self.copy_barplacement(N53,True,2)
        
        #N54 4 3 4 3排列
        barn54=AllplanGeo.Polyline3D()
        barn54+=AllplanGeo.Point3D(0,1600,0)
        barn54+=AllplanGeo.Point3D(0,-1600,0)
        offsetlist=[250,200,300,400]
        barcountlist=[5,8,8,8]
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=658,length=885,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,0,558/2+70))
        for i in range(4):
            shape.Move(AllplanGeo.Vector3D(offsetlist[i],0,0))
            N54=self.div_barplacement_on_polyline(96,shape,[0],[-200],[barcountlist[i]],barn54,1)
            reinforcement+=self.copy_barplacement(N54,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(150,0,558/2+70))
        for i in range(3):
            shape.Move(AllplanGeo.Vector3D(400,0,0))
            N54=self.div_barplacement_on_polyline(96,shape,[-100],[-200],[8],barn54,1)
            reinforcement+=self.copy_barplacement(N54,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,-45))
        shape.Move(AllplanGeo.Vector3D(0,0,558/2+20))
        N54=self.div_barplacement_on_polyline(96,shape,[450,300],[300,400],[1,2],self.barbottombottomlist)
        reinforcement+=self.copy_barplacement(N54,True,1)
        reinforcement=[]
        #N55
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=1040,length=1267,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,180))
        shape.Move(AllplanGeo.Vector3D(0,2730-1040/2,0))
        dislist=[6*150,150,300,150,300,150,300]
        for i in range(7):
            shape.Move(AllplanGeo.Vector3D(0,dislist[i]/self.q20*self.c20,dislist[i]))
            N55=self.div_barplacement_on_polyline(97,shape,[150,200],[100,200],[4,5],self.barbottombottomlist)
            reinforcement+=self.copy_barplacement(N55,True,2)
        #N56
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=374,length=601,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.bar_top_distance_list()[22],-374/2+30))
        N56=self.div_barplacement_on_polyline(98,shape,[1850,800],[500,500],[1,27],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N56,True,2)
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N45_steel(base=512,length=739,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(90,0,45))
        shape.Move(AllplanGeo.Vector3D(0,self.bar_top_distance_list()[26],-512/2+30))
        N56=self.div_barplacement_on_polyline(98,shape,[2450],[500],[28],self.bartoptoplist)
        reinforcement+=self.copy_barplacement(N56,True,2)
        return reinforcement

    def create_spiral_reinforcement(self):
        '''
        螺旋筋
        '''
        reinforcement=[]
        for i in range(2*self.waterholenum):
            N63_1=self.create_spiral(106,190,8,420,60,self.wateraxislist[i],z=self.height-420-50)
            reinforcement.append(N63_1)
        return reinforcement

    def create_spiral(self,pnum,radius,diameter,length,pitch,axis,x=0,y=0,z=0,bymeter=True,startloop=1,endloop=1):
        '''
        pnum:钢筋编号
        radius：钢筋环半径
        diameter：钢筋直径
        length：钢筋横向长度
        pitch：钢筋螺距
        axis：参照坐标轴
        x,y,x:钢筋偏移
        startloop：起始绕圈数
        endloop：结束绕圈数
        生成环形钢筋
        '''
        steelGrade=self.CirSteelGrade
        concreGrade=self.ConcreteGrade
        rotation_axis = AllplanGeo.Line3D(axis.CalcGlobalPoint(AllplanGeo.Point3D(0, 0, 0)),
                                          axis.CalcGlobalPoint(AllplanGeo.Point3D(0, 0, length+diameter*3)))
        rotation_axis=AllplanGeo.Move(rotation_axis,AllplanGeo.Vector3D(x,y,z))
        contour = AllplanGeo.Polyline3D()
        contour += axis.CalcGlobalPoint(AllplanGeo.Point3D(0, -radius/2, 0))
        contour += axis.CalcGlobalPoint(AllplanGeo.Point3D(0, -radius/2, length+diameter*3))
        contour=AllplanGeo.Move(contour,AllplanGeo.Vector3D(x,y,z))
        spiral = AllplanReinf.SpiralElement(pnum, diameter, steelGrade, concreGrade,
                                            rotation_axis, contour, pitch, 0,
                                            0, 0, 0,
                                            0, 0,  0)
        spiral.SetPlacePerLinearMeter(bymeter)
        spiral.SetNumberLoopsStart(startloop)
        spiral.SetNumberLoopsEnd(endloop)
        spiral.SetCommonProperties(self.com_prop)

        return spiral

    def polyline2d_to_3d(self,polyline2d,x=0,y=0,z=0,type=1):
        '''
        type:转换类型 为0：补充x轴坐标；为1：补充y轴坐标；其他：补充z轴坐标。
        2d点集转3d
        '''
        polyline3d=AllplanGeo.Polyline3D()
        for i in range(0,polyline2d.Count()):
            point2d=polyline2d[i]
            point3d=self.point2d_to_3d(point2d,x,y,z,type)
            polyline3d+=point3d
        return polyline3d

    def polyline3d_to_2d(self,polyline3d,type=1):
        '''
        type:省略类型 为0：省去x轴坐标；为1：省去y轴坐标；其他：省去z轴坐标。
        3d点集转2d
        '''
        polyline2d=AllplanGeo.Polyline2D()
        for i in range(0,polyline3d.Count()):
            point3d=polyline3d[i]
            point2d=self.point3d_to_2d(point3d,type)
            polyline2d+=point2d
        return polyline2d

    def point2d_to_3d(self,point2d,x=0,y=0,z=0,type=1):
        '''
        type:转换类型 为0：补充x轴坐标；为1：补充y轴坐标；其他：补充z轴坐标。
        2d点转3d点
        '''
        if type==0:
            y,z=point2d.GetCoords()
        elif type==1:
            x,z=point2d.GetCoords()
        else:
            x,y=point2d.GetCoords()
        return AllplanGeo.Point3D(x,y,z)

    def point3d_to_2d(self,point3d,type=1):
        '''
        type:省略类型 为0：省去x轴坐标；为1：省去y轴坐标；其他：省去z轴坐标。
        3d点转2d点
        '''
        x,y,z=point3d.GetCoords()
        if type==0:
            return AllplanGeo.Point2D(y,z)
        elif type==1:
            return AllplanGeo.Point2D(x,z)
        else:
            return AllplanGeo.Point2D(x,y)

    def polyline3d_offset(self,polyline3d,offset,x=0,y=0,z=0,type=1):
        '''
        type:省略补充类型 为0：省去补充x轴坐标；为1：省去补充y轴坐标；其他：省去补充z轴坐标。
        根据点集和偏移距离，生成新的点集
        '''
        polyline2d=self.polyline3d_to_2d(polyline3d,type)
        err1,newpolyline2d=AllplanGeo.Offset(offset,polyline2d,False)
        newpolyline3d=self.polyline2d_to_3d(newpolyline2d,x,y,z,type)
        return newpolyline3d

    def line3d_to_vector3d(self,line3d):
        '''
        3d线转3d向量
        '''
        return AllplanGeo.Vector3D(line3d.GetStartPoint(),line3d.GetEndPoint())

    def barplacement_on_line(self,positionNumber, shape,offset,distance,barcount,line3d,type=0):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        offset：起始偏移
        distance：钢筋间距在对应轴投影长度
        barcount：钢筋条数
        line3d：所沿铺设直线
        type:在哪个轴方向上布钢筋 0：沿x轴方向 1：沿y轴方向 其他：沿z轴方向
        生成钢筋沿直线线性铺设效果
        '''
        vector3d=self.line3d_to_vector3d(line3d)
        diameter=shape.GetDiameter()
        if type==0:
            xdz=vector3d.Z/vector3d.X
            xdy=vector3d.Y/vector3d.X
            start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(offset,offset*xdy,offset*xdz)
            distan=distance/vector3d.X*vector3d.GetLength()
        elif type==1:
            ydz=vector3d.Z/vector3d.Y
            ydx=vector3d.X/vector3d.Y
            start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(offset*ydx,offset,offset*ydz)
            distan=distance/vector3d.Y*vector3d.GetLength()
        else:
            zdx=vector3d.X/vector3d.Z
            zdy=vector3d.Y/vector3d.Z
            start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(offset*zdx,offset*zdy,offset)
            distan=distance/vector3d.Z*vector3d.GetLength()
        vector3d.Normalize(distan)
        position_point=start_point+vector3d
        barplacement=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                positionNumber, shape,
                start_point,
                position_point,
                -diameter/2,distan, barcount)
        return barplacement

    def barplacement_on_polyline(self,positionNumber, shape,offset,xdistance,barcount,polyline3d,type=0):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        offset：起始偏移
        xdistance：钢筋间距在对应轴投影长度
        barcount：钢筋条数
        polyline3d：所沿铺设折线
        type:在哪个轴方向上布钢筋 0：沿x轴方向 1：沿y轴方向 其他：沿z轴方向
        生成钢筋沿折线线性铺设效果(允许出线延伸铺设)
        '''
        barplacementlist=[]
        if type==0:
            startx=polyline3d[0].X+offset
            endx=polyline3d[0].X+offset+xdistance*(barcount-1)
            startindex,start_point=self.get_index_and_z_from_x_on_polyline(startx,polyline3d,type)
            endindex,end_point=self.get_index_and_z_from_x_on_polyline(endx,polyline3d,type)
            leftbarcount=barcount
            newoffset=startx-polyline3d[startindex].X
            for i in range(startindex,endindex+1):
                    line3d=polyline3d.GetLine(i)
                    vector3d=self.line3d_to_vector3d(line3d)
                    xdz=vector3d.Z/vector3d.X
                    xdy=vector3d.Y/vector3d.X
                    start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(newoffset,newoffset*xdy,newoffset*xdz)
                    distan=xdistance/vector3d.X*vector3d.GetLength()
                    vector3d.Normalize(distan)
                    position_point=start_point+vector3d
                    newbarcount=int((polyline3d[i+1].X-polyline3d[i].X-newoffset)/xdistance)+1
                    if(newbarcount>leftbarcount or i==endindex):
                        newbarcount=leftbarcount
                    enddx=(newbarcount-1)*xdistance+start_point.X
                    barplacement=self.barplacement_on_line(positionNumber,shape,newoffset,xdistance,newbarcount,line3d,type)
                    barplacementlist.append(barplacement)
                    newoffset=enddx+xdistance-polyline3d[i+1].X
                    leftbarcount-=newbarcount
        elif type==1:
            startx=polyline3d[0].Y+offset
            endx=polyline3d[0].Y+offset+xdistance*(barcount-1)
            startindex,start_point=self.get_index_and_z_from_x_on_polyline(startx,polyline3d,type)
            endindex,end_point=self.get_index_and_z_from_x_on_polyline(endx,polyline3d,type)
            leftbarcount=barcount
            newoffset=startx-polyline3d[startindex].Y
            for i in range(startindex,endindex+1):
                    line3d=polyline3d.GetLine(i)
                    vector3d=self.line3d_to_vector3d(line3d)
                    ydx=vector3d.X/vector3d.Y
                    ydz=vector3d.Z/vector3d.Y
                    start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(newoffset*ydx,newoffset,newoffset*ydz)
                    distan=xdistance/vector3d.Y*vector3d.GetLength()
                    vector3d.Normalize(distan)
                    position_point=start_point+vector3d
                    newbarcount=int((polyline3d[i+1].Y-polyline3d[i].Y-newoffset)/xdistance)+1
                    if(newbarcount>leftbarcount or i==endindex):
                        newbarcount=leftbarcount
                    enddx=(newbarcount-1)*xdistance+start_point.Y
                    barplacement=self.barplacement_on_line(positionNumber,shape,newoffset,xdistance,newbarcount,line3d,type)
                    barplacementlist.append(barplacement)
                    newoffset=enddx+xdistance-polyline3d[i+1].Y
                    leftbarcount-=newbarcount
        else:
            startx=polyline3d[0].Z+offset
            endx=polyline3d[0].Z+offset+xdistance*(barcount-1)
            startindex,start_point=self.get_index_and_z_from_x_on_polyline(startx,polyline3d,type)
            endindex,end_point=self.get_index_and_z_from_x_on_polyline(endx,polyline3d,type)
            leftbarcount=barcount
            newoffset=startx-polyline3d[startindex].Z
            for i in range(startindex,endindex+1):
                    line3d=polyline3d.GetLine(i)
                    vector3d=self.line3d_to_vector3d(line3d)
                    zdx=vector3d.X/vector3d.Z
                    zdy=vector3d.Y/vector3d.Z
                    start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(newoffset*zdx,newoffset*zdy,newoffset)
                    distan=xdistance/vector3d.Z*vector3d.GetLength()
                    vector3d.Normalize(distan)
                    position_point=start_point+vector3d
                    newbarcount=int((polyline3d[i+1].Z-polyline3d[i].Z-newoffset)/xdistance)+1
                    if(newbarcount>leftbarcount or i==endindex):
                        newbarcount=leftbarcount
                    enddx=(newbarcount-1)*xdistance+start_point.Z
                    barplacement=self.barplacement_on_line(positionNumber,shape,newoffset,xdistance,newbarcount,line3d,type)
                    barplacementlist.append(barplacement)
                    newoffset=enddx+xdistance-polyline3d[i+1].Z
                    leftbarcount-=newbarcount
        return barplacementlist

    def get_index_and_z_from_x_on_polyline(self,dis,polyline3d,type=0):
        '''
        dis:对应轴坐标
        polyline3d：3D折线
        type:对应何轴 0：对应x轴 1:对应y轴 其他:对应z轴
        根据对应轴坐标和在对应轴单向延伸的3D折线获取该坐标所对应的点及点所在的折线段数
        '''
        xlist=[]
        for i in range(0,polyline3d.Count()):
            if(type==0):
                xlist.append(polyline3d[i].X)
            elif(type==1):
                xlist.append(polyline3d[i].Y)
            else:
                xlist.append(polyline3d[i].Z)
        index=0
        for i in range(0,len(xlist)-1):
            if((dis-xlist[i])*(dis-xlist[i+1])<=0):
                break
            index+=1
        if(index==polyline3d.Count()-1):
            if type==0:
                if((dis-polyline3d[0].X)*(polyline3d[1].X-polyline3d[0].X)>0):
                    index=polyline3d.Count()-2
                else:
                    index=0
            elif type==1:
                if((dis-polyline3d[0].Y)*(polyline3d[1].Y-polyline3d[0].Y)>0):
                    index=polyline3d.Count()-2
                else:
                    index=0
            else:
                if((dis-polyline3d[0].Z)*(polyline3d[1].Z-polyline3d[0].Z)>0):
                    index=polyline3d.Count()-2
        offset=dis-xlist[index]
        line=polyline3d.GetLine(index)
        point=polyline3d[index]
        x=point.X
        y=point.Y
        z=point.Z
        vector=self.line3d_to_vector3d(line)
        if type==0:
            x+=offset
            y+=offset/vector.X*vector.Y
            z+=offset/vector.X*vector.Z
        elif type==1:
            x+=offset/vector.Y*vector.X
            y+=offset
            z+=offset/vector.Y*vector.Z
        else:
            x+=offset/vector.Z*vector.X
            y+=offset/vector.Z*vector.Y
            z+=offset
        return index,AllplanGeo.Point3D(x,y,z)

    def div_barplacement_on_polyline(self,positionNumber, shape,offsetlist,xdistancelist,barcountlist,polyline3d,type=0):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        offsetlist：起始偏移列表
        xdistance列表：钢筋间距在某轴投影长度列表
        barcount：钢筋条数列表
        polyline3d：所沿铺设折线
        type:对应何轴 0：对应x轴 1:对应y轴 其他:对应z轴
        生成同一种钢筋沿折线按不同区域不同间距线性铺设效果
        '''
        newbarplacementlist=[]
        totaloffset=0
        for j in range(0,len(offsetlist)):
            offset=offsetlist[j]
            totaloffset+=offset
            divdistan=xdistancelist[j]*(barcountlist[j]-1)
            barplace=self.barplacement_on_polyline(positionNumber,shape,totaloffset,xdistancelist[j],barcountlist[j],polyline3d,type)
            newbarplacementlist+=barplace
            totaloffset+=divdistan
        return newbarplacementlist

    def copy_barplacement(self,barplacement,linear,type):
        '''
        barplacement:需要镜像的barplacement(列表)
        linear:barplacement是否为线性铺设
        type:镜像方式 为0：关于xz平面镜像 为1：关于梁跨中心平面镜像 其他：同时关于两个平面镜像
        '''
        if not isinstance(barplacement, list):
            barplacement=[barplacement]
        barplacementlist=[]
        barplacementxz=[]
        barplacementx=[]
        barplacemento=[]
        for i in range(0,len(barplacement)):
            xz=self.reflection_barplacement(barplacement[i],linear,0)
            x=self.reflection_barplacement(barplacement[i],linear,1)
            o=self.reflection_barplacement(x,linear,0)
            barplacementxz.append(xz)
            barplacementx.append(x)
            barplacemento.append(o)
        barplacementlist+=barplacement
        if (type==0):
            barplacementlist+=barplacementxz
        elif (type==1):
            barplacementlist+=barplacementx
        else:
            barplacementlist+=barplacementxz
            barplacementlist+=barplacementx
            barplacementlist+=barplacemento
        return barplacementlist

    def line3d_to_polyline3d(self,line3d):
        '''
        line3d转polyline3d
        '''
        polyline3d=AllplanGeo.Polyline3D()
        polyline3d+=line3d.GetStartPoint()
        polyline3d+=line3d.GetEndPoint()
        return polyline3d

    def line3d_to_polyline3d(self,line3d):
        '''
        line3d转polyline3d
        '''
        polyline3d=AllplanGeo.Polyline3D()
        polyline3d+=line3d.GetStartPoint()
        polyline3d+=line3d.GetEndPoint()
        return polyline3d

    def polyline3d_reverse_longthen(self,polyline3d,type=0):
        '''
        type:反向延伸方向 为0:梁跨方向 其他：梁宽方向
        polyline3d关于指定方向反向延伸 
        '''
        matmg=AllplanGeo.Matrix3D()
        matmg.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(self.halflength, 0, 0),AllplanGeo.Point3D(self.halflength, 0, 1),AllplanGeo.Point3D(self.halflength, 1, 0)))
        matbg=AllplanGeo.Matrix3D()
        matbg.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        polyline=AllplanGeo.Polyline3D(polyline3d)
        if type==0:
            if polyline3d[polyline3d.Count()-1].X==self.halflength:
                for i in range(0,polyline3d.Count()-1):
                    point=polyline3d[polyline3d.Count()-1-1-i]
                    newpoint=AllplanGeo.Transform(point,matmg)
                    polyline+=newpoint
            else:
                for i in range(0,polyline3d.Count()):
                    point=polyline3d[polyline3d.Count()-1-i]
                    newpoint=AllplanGeo.Transform(point,matmg)
                    polyline+=newpoint
        else:
            if polyline3d[polyline3d.Count()-1].Y==0:
                for i in range(0,polyline3d.Count()-1):
                    point=polyline3d[polyline3d.Count()-1-1-i]
                    newpoint=AllplanGeo.Transform(point,matbg)
                    polyline+=newpoint
            else:
                for i in range(0,polyline3d.Count()):
                    point=polyline3d[polyline3d.Count()-1-i]
                    newpoint=AllplanGeo.Transform(point,matbg)
                    polyline+=newpoint
        return polyline

    def get_point_and_angle_from_offset_arc3d(self,arc3d,offset):
        '''
        求圆弧中点偏移后的位置以及垂直于偏移线的直线与x轴夹角
        '''
        middle=self.get_middle_point_of_arc3d(arc3d)
        center=arc3d.GetCenter()
        radius=arc3d.GetMajorRadius()
        line=AllplanGeo.Line3D(center,middle)
        line.TrimEnd(-offset)
        endpoint=line.GetEndPoint()
        center2d=self.point3d_to_2d(center,type=0)
        endpoint2d=self.point3d_to_2d(endpoint,type=0)
        angle=AllplanGeo.CalcAngle(center2d,endpoint2d)
        angledeg=90-angle.Deg
        return (endpoint,angledeg)

    def get_middle_point_of_arc3d(self,arc3d):
        '''
        返回圆弧的中点
        '''
        sa0=arc3d.GetStartAngle()
        sa0.Normalize2Pi()
        sa=sa0.Deg
        if sa==360:
            sa=0
        ea0=arc3d.GetEndAngle().Deg
        ma0=(sa+ea0)/2
        m0=AllplanGeo.Angle()
        m0.Deg=ma0
        return arc3d.GetPoint(m0)

    def get_polyline_from_point(self,point,type=0):
        '''
        type:点延伸方向 0：梁长方向延伸到底 1：梁宽方向延伸到底 2：梁高方向延伸到底
        生成该点沿指定方向延伸的折线
        '''
        polyline=AllplanGeo.Polyline3D()
        if type==0:
            endpoint=point+AllplanGeo.Vector3D(2*self.halflength,0,0)
        elif type==1:
            endpoint=point+AllplanGeo.Vector3D(0,2*self.halfwidth,0)
        else:
            endpoint=point+AllplanGeo.Vector3D(0,0,height)
        polyline+=point
        polyline+=endpoint
        return polyline

    def vector_on_slope(self,angle,x,y):
        '''
        angle为yz平面上斜坡（起点为原点）与x轴正方向夹角（斜向下为正）,
        x为沿斜坡横向移动距离
        y为沿斜坡纵向移动距离
        生成在斜坡坐标为（x,y）的点的真实偏移向量
        '''
        vector1=AllplanGeo.Vector3D(0,x*math.cos(angle),x*math.sin(angle))
        vector2=AllplanGeo.Vector3D(0,-y*math.sin(angle),y*math.cos(angle))
        vector=vector1+vector2
        return vector

    def get_polyline_from_lines(self,line1,line2):
        '''
        两线段，第一条线段延伸一定距离，第二条线段反向延伸一定距离，求交点，返回相交后的折线;若无交点，返回两线段起点终点连线
        '''
        line1.TrimEnd(-50000)
        line2.TrimStart(-50000)
        bool,intersec=AllplanGeo.IntersectionCalculus(line1,line2)
        polyline=AllplanGeo.Polyline3D()
        polyline+=line1.StartPoint
        if bool:
            polyline+=intersec
        polyline+=line2.EndPoint
        return polyline

    def get_middle_of_line_intersected_in_lines(self,line,line1,line2):
        '''
        求夹在两线段中间的线段两头延伸一段距离后与两线段的两交点的中点及该线段
        '''
        poly1=self.get_polyline_from_lines(line1,line)
        poly2=self.get_polyline_from_lines(line,line2)
        newline=AllplanGeo.Line3D(poly1[1],poly2[1])
        middle=newline.GetCenterPoint()
        return newline,middle

    def create_ring_runway(self,halflength,radius,height):
        '''
        length:总半长
        radius：跑道半径和宽度
        height：向上延伸高度
        生成环形跑道型立体模型
        '''
        path=AllplanGeo.Path3D()
        centerpoint1=AllplanGeo.Point3D(0,halflength-radius,0)
        arc3d1=AllplanGeo.Arc3D(centerpoint1,AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,0,1),radius,radius,0,math.pi,True)
        centerpoint2=AllplanGeo.Point3D(0,-halflength+radius,0)
        arc3d2=AllplanGeo.Arc3D(centerpoint2,AllplanGeo.Vector3D(-1,0,0),AllplanGeo.Vector3D(0,0,1),radius,radius,0,math.pi,True)
        line1=AllplanGeo.Line3D(-radius,halflength-radius,0,-radius,-halflength+radius,0)
        line2=AllplanGeo.Line3D(radius,-halflength+radius,0,radius,halflength-radius,0)
        path+=arc3d1
        path+=line1
        path+=arc3d2
        path+=line2
        line=AllplanGeo.Line3D(radius,halflength-radius,0,radius,halflength-radius,height)
        err,ringrunway=AllplanGeo.CreateSweptBRep3D([path],line,True,False,None,0)
        if not err:
            return (err,ringrunway)
        else:
            return (err,0)

    def bar_distance_list(self):
        '''
        梁顶面钢筋距离列表
        '''
        list=[]
        list.append(-60)
        list.append(-145)
        for i in range(0,17):
            list.append(-150)
        list.append(-113)
        for i in range(0,3):
            list.append(-111)
        for i in range(0,2):
            list.append(-107)
        for i in range(0,4):
            list.append(-100)
        for i in range(0,3):
            list.append(-125)
        list.append(-110)
        list.append(-100)
        list.append(-140)
        for i in range(0,2):
            list.append(-80)
        for i in range(0,32):
            list.append(-100)
        return list

    def bar_total_distance_list(self):
        '''
        梁顶板钢筋总计距离列表
        '''
        list=self.bar_distance_list()
        newlist=[]
        distance=0
        for i in range(0,len(list)):
            distance+=list[i]
            newlist.append(distance)
        return newlist
 
    def distancelist_barplacement_on_polyline(self,positionNumber, shape,distancelist,startcount,count,polyline3d,type=0,step=1):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        distancelist：总偏移列表
        startcount:第一根钢筋所在位置编号（对应总偏移列表）
        endcount:最后一根钢筋所在位置编号（对应总偏移列表）
        polyline3d：所沿铺设折线
        type:对应何轴 0：对应x轴 1:对应y轴 其他:对应z轴
        step:间隔根数，1根为无间隔，2根为中间隔1根
        生成同一种钢筋沿折线按不同区域不同间距线性铺设效果
        '''
        newbarplacementlist=[]
        totaloffset=0
        for i in range(count):
            barplace=self.div_barplacement_on_polyline(positionNumber,shape,[distancelist[startcount+i*step]],[100],[1],polyline3d,type)
            newbarplacementlist+=barplace
        return newbarplacementlist

    def bar_bottom_distance_list(self):
        '''
        底板底层钢筋总偏移列表
        '''
        list=[0]
        distance=0
        for i in range(16):
            distance+=100
            list.append(distance)
        distance+=113
        list.append(distance)
        for i in range(4):
            distance+=110
            list.append(distance)
        for i in range(2):
            distance+=107
            list.append(distance)
        for i in range(3):
            distance+=111
            list.append(distance)
        return list

    def bar_top_distance_list(self):
        '''
        顶板钢筋总偏移列表
        '''
        list=[0]
        distance=0
        for i in range(16):
            distance+=100
            list.append(distance)
        distance+=113
        list.append(distance)
        for i in range(2):
            distance+=80
            list.append(distance)
        distance+=140
        list.append(distance)
        distance+=100
        list.append(distance)
        distance+=110
        list.append(distance)
        for i in range(3):
            distance+=125
            list.append(distance)
        for i in range(4):
            distance+=100
            list.append(distance)
        for i in range(2):
            distance+=107
            list.append(distance)
        for i in range(3):
            distance+=111
            list.append(distance)
        distance+=113
        list.append(distance)
        for i in range(17):
            distance+=150
            list.append(distance)
        distance+=145
        list.append(distance)
        distance+=60
        list.append(distance)
        return list

    def bar_height_distance_list(self):
        '''
        竖直方向总偏移列表
        '''
        list=[65]
        distance=0
        for i in range(11):
            distance+=150
            list.append(distance)
        distance+=108
        list.append(distance)
        distance+=154
        list.append(distance)
        for i in range(2):
            distance+=169
            list.append(distance)
        distance+=141
        list.append(distance)
        distance+=362
        list.append(distance)
        distance+=56
        list.append(distance)
        distance+=123
        list.append(distance)
        return list

    def string_to_list(self,string):
        '''
        以空格为间隔的数字组成的字符串转换成列表
        '''
        list=string.split(" ")
        for i in range(len(list)):
            list[i]=eval(list[i])
        return list
