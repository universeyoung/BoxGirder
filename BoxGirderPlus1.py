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
        polyhedron.append(spiral)
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
        #镜像生成另一半实体
        matgir=AllplanGeo.Matrix3D()
        matgir.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(secdislist[self.secnum-1], 0, 0),AllplanGeo.Point3D(secdislist[self.secnum-1], 0, 1),AllplanGeo.Point3D(secdislist[self.secnum-1], 1, 0)))
        halfGirderPlus1=AllplanGeo.Transform(halfGirderPlus,matgir)
        #合并
        errcomb,GirderPlus=AllplanGeo.MakeUnion(halfGirderPlus,halfGirderPlus1)
        #泄水孔
        swhcy=[]
        if self.waterholeaway==1:
            swhx=self.waxdis
        else:
            swhx=halflength-self.waxdis
        if self.waterholeawayy==0:
            swhy=self.waydis
        else:
            swhy=halfwidth-self.waydis
        for i in range(0,self.waterholenum):
            axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(swhx+i*self.wahdis,swhy,0),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,0,1))
            swh=AllplanGeo.BRep3D.CreateCylinder(axis,self.waterholediameter,height)
            rswh=AllplanGeo.Transform(swh,matxz)
            swhcy.append(swh)
            swhcy.append(rswh)
        if self.waterholeline>2:
            if self.waterholemaway==1:
                mwhx=self.wamxdis
            else:
                mwhx=halflength-self.wamxdis
            if self.waterholemawayy==0:
                mwhy=self.wamydis
            else:
                mwhy=halfwidth-self.wamydis
            for i in range(0,self.waterholemnum):
                axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(mwhx+i*self.wahmdis,mwhy,height/2),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,0,1))
                mwh=AllplanGeo.BRep3D.CreateCylinder(axis,self.waterholediameter,height/2)
                swhcy.append(mwh)
        #挖孔
        for i in range(0,len(swhcy)):
            errsub,GirderPlus=AllplanGeo.MakeSubtraction(GirderPlus,swhcy[i])
        #通风孔
        swihcy=[]
        if self.windholeaway==1:
            swihx=self.wixdis
        else:
            swihx=halflength-self.wixdis
        if self.windholeawayy==1:
            swihz=self.wiydis
        else:
            swihz=height-self.wiydis
        for i in range(0,self.windholenum):
            axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(swihx+i*self.wihdis,-halfwidth,swihz),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,1,0))
            swih=AllplanGeo.BRep3D.CreateCylinder(axis,self.windholediameter,2*halfwidth)
            swihcy.append(swih)
        if self.windholeline>1:
            if self.windholemaway==1:
                mwihx=self.wimxdis
            else:
                mwihx=halflength-self.wimxdis
            if self.windholemawayy==1:
                mwihz=self.wimydis
            else:
                mwihz=height-self.wimydis
            for i in range(0,self.windholemnum):
                axis=AllplanGeo.AxisPlacement3D(AllplanGeo.Point3D(mwihx+i*self.wihmdis,-halfwidth,mwihz),AllplanGeo.Vector3D(1,0,0),AllplanGeo.Vector3D(0,1,0))
                mwih=AllplanGeo.BRep3D.CreateCylinder(axis,self.windholediameter,2*halfwidth)
                swihcy.append(mwih)
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
                inupcharm,angle1=self.get_point_and_angle_from_offset_arc3d(inupcharmfer,30)
                self.barinupcharmlist+=inupcharm
                self.angle1list.append(angle1)
            if(indowncharmfer!=0):
                indowncharm,angle2=self.get_point_and_angle_from_offset_arc3d(indowncharmfer,30)
                self.barindowncharmlist+=indowncharm
                self.angle2list.append(angle2)
        #生成顶板顶层中点列表和底板底层中点列表
        self.toptoplist=AllplanGeo.Line3D(0,0,height,2*halflength,0,height)
        self.bottombottomlist=AllplanGeo.Line3D(0,0,0,2*halflength,0,0)
        #生成铺设线的偏移后实际折线
        self.bartopmidlist=self.polyline3d_offset(self.topmidlist,30)
        self.barbottommidlist=self.polyline3d_offset(self.bottommidlist,-35)
        self.bartopmidlist=self.polyline3d_reverse_longthen(self.bartopmidlist)
        self.barbottommidlist=self.polyline3d_reverse_longthen(self.barbottommidlist)
        self.bartoptoplist=AllplanGeo.Line3D(0,0,height,2*halflength,0,height)
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
        point=self.get_middle_of_line_intersected_in_lines(line,line1,line2)
        self.barn15=self.get_polyline_from_point(point)
        self.barn10=AllplanGeo.Polyline3D()
        for i in range(0,self.secnum):
            newline1=AllplanGeo.Move(line1,AllplanGeo.Vector3D(secdislist[i],0,0))
            newline2=AllplanGeo.Move(line2,AllplanGeo.Vector3D(secdislist[i],0,0))
            newline=AllplanGeo.Line3D(midlist[i][1],midlist[i][0])
            newpoint=self.get_middle_of_line_intersected_in_lines(newline,newline1,newline2)
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
        matxz=AllplanGeo.Matrix3D()
        matxz.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(0, 0, 0),AllplanGeo.Point3D(0, 0, 1),AllplanGeo.Point3D(1, 0, 0)))
        outsect,outmain,outscale=self.create_common_outer_section_path()
        start_point=outmain[3].GetEndPoint()
        end_point=start_point+AllplanGeo.Vector3D(24600,0,0)
        angle_global_to_local     = AllplanGeo.Angle()
        angle_global_to_local.Deg = -90
        reinforce=[]
        hooklength=((2765-155-2001-355)/2.0)
        starthookangle=135
        endhookangle=135
        r=4
        #N1 N5 N7 N8为顶板顶层横向钢筋
        #N1系 7根N1 1根N1-2 9根N1 1根N1-2 7根N1 起始偏移50 钢筋间距100 1根N1-2两边各配1根N1-1 关于梁跨中心镜像 铺设线：顶板顶层梁体中心线
        #总计 （7+9+7）*2=46根N1 （1+1）*2=4根N1-2 4*2=8根N1-1
        #N1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,-35-22/2))
        start_point=outmain[0].GetStartPoint()

        dislistn1=[50,200,200]
        barcountlistn1=[7,9,7]
        for i in range(0,len(dislistn1)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn1[i],0,0)
            position_point=start_point+AllplanGeo.Vector3D(100,0,0)
            N1=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    1, shape,
                    start_point,
                    position_point,
                    -22/2, 100, barcountlistn1[i])
            reinforce.append(N1)
            N1c=self.reflection_barplacement(N1,True,1)
            reinforce.append(N1c)
            start_point=N1.GetEndPoint()
        #N1-2 N1中混杂的两根
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,-35-22/2))
        start_point=outmain[0].GetStartPoint()

        start_point=start_point+AllplanGeo.Vector3D((8-1)*100+50,0,0)
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N1_2=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    3, shape,
                    start_point,
                    position_point,
                    -22/2, 100*10,2)
        reinforce.append(N1_2)
        N1_2c=self.reflection_barplacement(N1_2,True,1)
        reinforce.append(N1_2c)
        
        #N1-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,7600/2-1895,-35-22/2))
        start_point=outmain[0].GetStartPoint()

        start_point=start_point+AllplanGeo.Vector3D((8-1)*100+50,0,0)
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N1_1=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    2, shape,
                    start_point,
                    position_point,
                    -22/2, 100*10,2)
        reinforce.append(N1_1)
        N1_1c=self.reflection_barplacement(N1_1,True,1)
        reinforce.append(N1_1c)
        N1_1r=self.reflection_barplacement(N1_1,True,0)
        reinforce.append(N1_1r)
        N1_1cr=self.reflection_barplacement(N1_1r,True,1)
        reinforce.append(N1_1cr)
        

        #N5
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_steel(diameter=18)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,-35-18/2))
        start_point=outmain[0].GetStartPoint()

        dislistn5=[self.halflength-94*125-20*100,100+125*2,125*2,125*2,125*2,125*2,125*2,125*2,100+125*2]
        barcountlistn5=[20,93-80-1,80-40-1,40-26-1,26*2-1,40-26-1,80-40-1,93-80-1,20]
        bardislistn5=[100,125,125,125,125,125,125,125,100]
        for i in range(0,len(dislistn5)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn5[i],0,0)
            position_point=start_point+AllplanGeo.Vector3D(125,0,0)
            N5=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    10, shape,
                    start_point,
                    position_point,
                    -18/2, bardislistn5[i], barcountlistn5[i])
            reinforce.append(N5)
            start_point=N5.GetEndPoint()

        #N5-2 N5中混杂的4根
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N5_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,-35-18/2))
        start_point=outmain[0].GetStartPoint()

        start_point=start_point+AllplanGeo.Vector3D(self.halflength-80*125,0,0)
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N5_2=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    12, shape,
                    start_point,
                    position_point,
                    -18/2, 125*(80-26),2)
        reinforce.append(N5_2)
        N5_2c=self.reflection_barplacement(N5_2,True,1)
        reinforce.append(N5_2c)

        #N5-1
        
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N5_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,-30-16/2,-35-16/2))
        start_point=outmain[4].GetStartPoint()

        dislistn5_1=[50+22/2+18/2,100,125*2,125*2,125*2,125*2,100-22/2-18/2-22/2]
        barcountlistn5_1=[45,94-80,80-26-1,26*2-1,80-26-1,94-80,45]
        bardislistn5_1=[100,125,125,125,125,125,100]
        for i in range(0,len(dislistn5_1)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn5_1[i],0,0)
            position_point=start_point+AllplanGeo.Vector3D(125,0,0)
            N5_1=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    11, shape,
                    start_point,
                    position_point,
                    -18/2, bardislistn5_1[i], barcountlistn5_1[i])
            reinforce.append(N5_1)
            N5_1r=self.reflection_barplacement(N5_1,True,0)
            reinforce.append(N5_1r)
            start_point=N5_1.GetEndPoint()
        #N5-3  配套N5-2
        
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N5_3_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,-30-18/2,-35-18/2))
        start_point=outmain[4].GetStartPoint()

        start_point=start_point+AllplanGeo.Vector3D(self.halflength-80*125,0,0)
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N5_3=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    13, shape,
                    start_point,
                    position_point,
                    -18/2, 125*(80-26),2)
        reinforce.append(N5_3)
        N5_3c=self.reflection_barplacement(N5_3,True,1)
        reinforce.append(N5_3c)
        N5_3r=self.reflection_barplacement(N5_3,True,0)
        reinforce.append(N5_3r)
        N5_3cr=self.reflection_barplacement(N5_3r,True,1)
        reinforce.append(N5_3cr)
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
        start_point=outmain[0].GetStartPoint()

        dislistn5_4=[4550,(93-40)*125]
        barcountlistn5_4=[2,1]
        bardislistn5_4=[125,125]
        for i in range(0,len(dislistn5_4)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn5_4[i],0,0)
            position_point=start_point+AllplanGeo.Vector3D(125,0,0)
            N5_4=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    14, shape,
                    start_point,
                    position_point,
                    -18/2, bardislistn5_4[i], barcountlistn5_4[i])
            reinforce.append(N5_4)
            N5_4r=self.reflection_barplacement(N5_4,True,0)
            reinforce.append(N5_4r)
            N5_4c=self.reflection_barplacement(N5_4,True,1)
            reinforce.append(N5_4c)
            N5_4cr=self.reflection_barplacement(N5_4r,True,1)
            reinforce.append(N5_4cr)
            start_point=N5_4.GetEndPoint()
        #N7
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N7_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,7600/2-3700,-35-18/2))
        start_point=outmain[0].GetStartPoint()
        dislistn7=[self.halflength-18-23*4*125,2*125]
        barcountlistn7=[1,23]
        bardislistn7=[125*2,125*4]
        for i in range(0,len(dislistn7)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn7[i],0,0)
            position_point=start_point+AllplanGeo.Vector3D(125,0,0)
            N7=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    18, shape,
                    start_point,
                    position_point,
                    -18/2, bardislistn7[i], barcountlistn7[i])
            reinforce.append(N7)
            N7c=self.reflection_barplacement(N7,True,1)
            reinforce.append(N7c)
            N7c.Move(AllplanGeo.Vector3D(18,0,0))
            start_point=N7.GetEndPoint()
        
        #N8
        profiles,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N7_steel(a=498,b=705,c=3170)
        profilee,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N7_steel(a=212,b=300,c=3742)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
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
        reinforce.append(N8)
        N8c=self.reflection_barplacement(N8,False,1)
        reinforce.append(N8c)
        #N2 N14为顶板底层横向钢筋
        #N2-2 2根 N2 4根 （N1-2配N2-1两根）1组 N210根 （N1-2配N2-1两根）1组 N2 7根
        #N2-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_2_steel()
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape_mat = AllplanGeo.Matrix3D()
        shape_mat.SetRotation(AllplanGeo.Line3D(0,0,0,0,1000,0), angle_global_to_local)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,22/2))
        start_point=self.bartopmidlist[0]
        position_point=start_point+AllplanGeo.Vector3D(100,0,0)
        N2_2=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    7, shape,
                    start_point,
                    position_point,
                    50-22/2, 100,2)
        reinforce.append(N2_2)
        N2_2c=self.reflection_barplacement(N2_2,True,1)
        reinforce.append(N2_2c)
        #N2
        '''
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
        dislistn2=[50+2*100,2*100]
        barcountlistn2=[5,7]
        bardislistn2=[100,100]
        for i in range(0,len(dislistn2)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn2[i],0,0)
            position_point=start_point+AllplanGeo.Vector3D(100,0,0)
            N2=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    22, shape,
                    start_point,
                    position_point,
                    -22/2, bardislistn2[i], barcountlistn2[i])
            reinforce.append(N2)
            N2c=self.reflection_barplacement(N2,True,1)
            reinforce.append(N2c)
            start_point=N2.GetEndPoint()
        barvec=self.line3d_to_vector3d(self.bartopmidlist.GetLine(1))
        barvec0=self.line3d_to_vector3d(self.bartopmidlist.GetLine(0))
        start_point=self.bartopmidlist[0]+AllplanGeo.Vector3D(1550,0,(1550-barvec0.GetLength())*barvec.Z/barvec.X)
        dislistn2=[0,2*100]
        barcountlistn2=[2,7]
        bardislistn2=[100/barvec.X*barvec.GetLength(),100/barvec.X*barvec.GetLength()]
        for i in range(0,len(dislistn2)):
            start_point=start_point+AllplanGeo.Vector3D(dislistn2[i],0,dislistn2[i]*barvec.Z/barvec.X)
            position_point=start_point+AllplanGeo.Vector3D(100,0,100*barvec.Z/barvec.X)
            N2=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                    22, shape,
                    start_point,
                    position_point,
                    -22/2, bardislistn2[i], barcountlistn2[i])
            reinforce.append(N2)
            N2c=self.reflection_barplacement(N2,True,1)
            reinforce.append(N2c)
            start_point=N2.GetEndPoint()
        '''
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
        reinforce+=self.copy_barplacement(N2,True,1)
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
        reinforce.append(N1_21)
        N1_21.Move(AllplanGeo.Vector3D(-22,0,0))
        N1_21c=self.reflection_barplacement(N1_21,True,1)
        reinforce.append(N1_21c)
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
            reinforce.append(N1_22)
            N1_22c=self.reflection_barplacement(N1_22,True,1)
            reinforce.append(N1_22c)
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
        reinforce+=self.copy_barplacement(N2_1,True,2)
        N2_2=self.get_combine_barplacement(N1_22,5,shape)
        reinforce+=self.copy_barplacement(N2_2,True,2)
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
        reinforce+=self.copy_barplacement(N14_1,True,1)
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
        reinforce+=self.copy_barplacement(N14_2,True,1)
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
        reinforce+=self.copy_barplacement(N14,True,1)
        shape.Move(AllplanGeo.Vector3D(0,0,-62))
        N14=self.div_barplacement_on_polyline(27,shape,[50],[100],[3],self.bartoptoplist)
        reinforce+=self.copy_barplacement(N14,True,1)
        shape.Move(AllplanGeo.Vector3D(0,0,-62))
        N14=self.div_barplacement_on_polyline(27,shape,[150],[100],[2],self.bartoptoplist)
        reinforce+=self.copy_barplacement(N14,True,1)
        shape.Move(AllplanGeo.Vector3D(0,0,-62))
        N14=self.div_barplacement_on_polyline(27,shape,[50],[100],[20],self.bartoptoplist)
        reinforce+=self.copy_barplacement(N14,True,1)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,0,8))
        N14=self.div_barplacement_on_polyline(27,shape,[2550,100+2*125,2*125,2*125,100+2*125],[100,125,125,125,100],[20,52,79,52,20],self.bartopmidlist)
        reinforce+=N14
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
        reinforce+=self.copy_barplacement(N14_3,True,2)
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
        reinforce+=self.copy_barplacement(N44_2,True,2)
        
        #N44 倒角钢筋
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=600,length=790,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-self.angle1list[2],0,0))
        N44=self.div_barplacement_on_polyline(82,shape,[2550,125,125],[100,125,100],[21,94*2-1,21],self.barinupcharmlist)
        reinforce+=self.copy_barplacement(N44,True,0)
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
        reinforce+=self.copy_barplacement(N31,True,0)
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
        shape.Move(self.vector_on_slope(angle,-(2850/2-300),35+12/2))
        wing=outmain[6].GetStartPoint()
        wingpoly=self.get_polyline_from_point(wing)
        N32=self.div_barplacement_on_polyline(64,shape,[50],[100],[24],wingpoly)
        reinforce+=self.copy_barplacement(N32,True,2)
        #N33
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N33_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        N33=self.div_barplacement_on_polyline(65,shape,[2450-self.dis03,125,2*125,2*125,2*125,2*125,125],[100,125,125,125,125,125,100],[22,13,53,51,53,13,22],self.n33barline)
        reinforce+=self.copy_barplacement(N33,True,0)
        #N33-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N33_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        N33_1=self.div_barplacement_on_polyline(66,shape,[self.halflength-self.dis03-80*125,(80-26)*125],[125,125],[1,1],self.n33barline)
        reinforce+=self.copy_barplacement(N33_1,True,2)
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
        reinforce+=self.copy_barplacement(N33_2,True,2)
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
        reinforce+=self.copy_barplacement(N44_3,True,2)
        #N44-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=900,length=1090,diameter=12)
        shape_props = ReinforcementShapeProperties.rebar(12,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(-anglen44,0,0))
        N44_1=self.div_barplacement_on_polyline(83,shape,[150,125,100],[100,125,100],[45,187,45],n44bar)
        reinforce+=self.copy_barplacement(N44_1,True,0) 
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
        reinforce+=N12
        #N12-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N1_2_steel(base=1185,down=180,length=1831,diameter=18)
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N12_2=self.div_barplacement_on_polyline(26,shape,[4550,13*125,5000,1750],[125,125,125,125],[2,1,1,1],self.barbottombottomlist)
        reinforce+=self.copy_barplacement(N12_2,True,1)
        #N12-1 搭配N12-2
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N12_1_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,-1,starthookangle,endhookangle)
        shape.Move(AllplanGeo.Vector3D(0,5032/2-1700,0))
        N12_1=self.get_combine_barplacement(N12_2,25,shape)
        reinforce+=self.copy_barplacement(N12_1,True,2)
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
        reinforce+=self.copy_barplacement(N6,True,1)
        #N6-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N6_steel(a=72,b=5344,c=2996,d=2906,e=727,f=200,r=54,length=11880,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Freeform)
        shapen6_1 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,-1,starthookangle,endhookangle)
        N6_1=self.div_barplacement_on_polyline(16,shapen6_1,[350],[100],[12],self.barbottombottomlist)
        reinforce+=self.copy_barplacement(N6_1,True,1)
        #N6-2
        shapen6_1.Move(AllplanGeo.Vector3D(1550,0,50))
        shapen6.Move(AllplanGeo.Vector3D(2050,0,50))
        N6_2=AllplanReinf.BarPlacement(17,6,shapen6_1,shapen6)
        reinforce+=self.copy_barplacement(N6_2,False,1)
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
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[3],self.barn15)
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-160,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[3],self.barn15)
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[16],self.barn15)
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[50],[100],[4],self.barn15)
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[150],[100],[3],self.barn15)
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-160,0))
        N15=self.div_barplacement_on_polyline(31,shape,[750],[100],[16],self.barn15)
        reinforce+=self.copy_barplacement(N15,True,2)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        angle=math.atan(self.q20/self.c20)*180/math.pi
        shape.Rotate(RotationAngles(angle,0,0))
        N15=self.div_barplacement_on_polyline(31,shape,[4550],[125],[95*2-1],self.barn10)
        reinforce+=self.copy_barplacement(N15,True,0)
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
        reinforce+=self.copy_barplacement(N15,True,2)
        shape.Move(AllplanGeo.Vector3D(0,-80,0))
        N15=self.div_barplacement_on_polyline(31,shape,[-800],[100],[2],barn15sp)
        reinforce+=self.copy_barplacement(N15,True,2)
        #N10
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=3060,length=3376,diameter=20)
        shape_props = ReinforcementShapeProperties.rebar(20,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(angle,0,0))
        N10=self.div_barplacement_on_polyline(22,shape,[350],[100],[42],self.barn10)
        reinforce+=self.copy_barplacement(N10,True,2)
        #N10-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=2490,length=2838,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(angle,0,0))
        N10_1=self.div_barplacement_on_polyline(23,shape,[71],[100],[23],self.barn10)
        reinforce+=self.copy_barplacement(N10_1,True,2)
        '''
        底板顶层N4系N3系N37系N35系
        N4系：起始偏移50 N4 间距100 3根 铺设同时关于梁跨中心和梁体中心对称
        N3系：起始偏移350 N3-1 1根 偏移100 接N3 间距100 13根 铺设关于梁跨中心对称
        N37系：起始偏移1750 N37 间距100 29根 前9根两根一组 铺设关于梁跨中心对称
        N35系：起始偏移4650 间距125 N35-2 2根 N35 20根 N35-2 1根 N35 31根  N35-2 1根 N35 31根 N35-2 1根 N35 8根铺设关于梁跨中心对称 1根N35-2配2根35-1
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
        reinforce=[]
        reinforce+=self.copy_barplacement(N4,True,2)
        
        #N3-1
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=3000,length=3348,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N3_1=self.div_barplacement_on_polyline(9,shape,[350],[100],[1],self.barbottommidlist)
        reinforce+=self.copy_barplacement(N3_1,True,1)
        #N3
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5704,length=6102,diameter=22)
        shape_props = ReinforcementShapeProperties.rebar(22,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape.Rotate(RotationAngles(180,0,0))
        N3=self.div_barplacement_on_polyline(9,shape,[450],[100],[13],self.barbottommidlist)
        reinforce+=self.copy_barplacement(N3,True,1)
        #N37
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551,length=5837-98,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape1 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape1.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551+17*7,length=5837-98+17*7,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape2 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape2.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551+18*7,length=5837-98+18*7,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape5 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape5.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5747,length=5837+98,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape6 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape6.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551+7*8,length=5837-98+7*8,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape3 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape3.Rotate(RotationAngles(180,0,0))
        profile,hooklength,starthookangle,endhookangle=Createsteelshape.shape_N2_steel(base=5551,length=5837-98,diameter=16)
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.LongitudinalBar)
        shape4 = ProfileShapeBuilder.create_profile_shape(profile,
                                                           shape_mat,
                                                           shape_props,
                                                           0,hooklength,hooklength,starthookangle,endhookangle)
        shape4.Rotate(RotationAngles(180,0,0))
        index,point1=self.get_index_and_z_from_x_on_polyline(1750,self.barbottommidlist)
        shape1.Move(AllplanGeo.Vector3D(point1))
        index,point2=self.get_index_and_z_from_x_on_polyline(3450,self.barbottommidlist)
        shape2.Move(AllplanGeo.Vector3D(point2))
        N37=AllplanReinf.BarPlacement(9,18,shape1,shape2)
        reinforce+=self.copy_barplacement(N37,False,1)
        index,point3=self.get_index_and_z_from_x_on_polyline(1750+22,self.barbottommidlist)
        shape4.Move(AllplanGeo.Vector3D(point3))
        index,point4=self.get_index_and_z_from_x_on_polyline(2550+22,self.barbottommidlist)
        shape3.Move(AllplanGeo.Vector3D(point4))
        N37=AllplanReinf.BarPlacement(9,9,shape4,shape3)
        reinforce+=self.copy_barplacement(N37,False,1)
        
        index,point5=self.get_index_and_z_from_x_on_polyline(3550,self.barbottommidlist)
        shape5.Move(AllplanGeo.Vector3D(point5))
        index,point6=self.get_index_and_z_from_x_on_polyline(4550,self.barbottommidlist)
        shape6.Move(AllplanGeo.Vector3D(point6))
        N37=AllplanReinf.BarPlacement(9,11,shape5,shape6)
        reinforce+=self.copy_barplacement(N37,False,1)
        #N16
        profile,hooklength=Createsteelshape.shape_N16_steel()
        shape_props = ReinforcementShapeProperties.rebar(18,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
        stirrup = AllplanReinf.StirrupType.FullCircle
        shape = ProfileShapeBuilder.create_profile_stirrup(profile,
                                                           shape_mat,
                                                           shape_props,0,stirrup)
        shape.Move(AllplanGeo.Vector3D(0,0,-35-22/2))
        start_point=outmain[0].GetStartPoint()
        position_point=start_point+AllplanGeo.Vector3D(1000,0,0)
        N16=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                32, shape,
                start_point,
                position_point,
                50-22/2, 100, 24)
        return reinforce

    def create_spiral_reinforcement(self):
        '''
        生成环形钢筋
        '''
        #N57
        rotation_axis = AllplanGeo.Line3D(AllplanGeo.Point3D(0, 0, 0),
                                          AllplanGeo.Point3D(0, 0, 330+8*3))

        contour = AllplanGeo.Polyline3D()
        contour += AllplanGeo.Point3D(0, -170/2, 0)
        contour += AllplanGeo.Point3D(0, -170/2, 330+8*3)

        spiral = AllplanReinf.SpiralElement(3, 8, 10, 20,
                                            rotation_axis, contour, 60, 0,
                                            0, 0, 0,
                                            0, 0,  0)
        spiral.SetPlacePerLinearMeter(True)
        spiral.SetNumberLoopsStart(1)
        spiral.SetNumberLoopsEnd(1)
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

    def barplacement_on_line(self,positionNumber, shape,offset,xdistance,barcount,line3d):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        offset：起始偏移
        xdistance：钢筋间距在x轴投影长度
        barcount：钢筋条数
        line3d：所沿铺设直线
        生成钢筋沿直线线性铺设效果
        '''
        vector3d=self.line3d_to_vector3d(line3d)
        xdz=vector3d.Z/vector3d.X
        xdy=vector3d.Y/vector3d.X
        start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(offset,offset*xdy,offset*xdz)
        diameter=shape.GetDiameter()
        distan=xdistance/vector3d.X*vector3d.GetLength()
        vector3d.Normalize(distan)
        position_point=start_point+vector3d
        barplacement=LinearBarBuilder.create_linear_bar_placement_from_by_dist_count(
                positionNumber, shape,
                start_point,
                position_point,
                -diameter/2,distan, barcount)
        return barplacement

    def barplacement_on_polyline(self,positionNumber, shape,offset,xdistance,barcount,polyline3d):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        offset：起始偏移
        xdistance：钢筋间距在x轴投影长度
        barcount：钢筋条数
        polyline3d：所沿铺设折线
        生成钢筋沿折线线性铺设效果(允许出线延伸铺设)
        '''
        startx=polyline3d[0].X+offset
        endx=polyline3d[0].X+offset+xdistance*(barcount-1)
        startindex,start_point=self.get_index_and_z_from_x_on_polyline(startx,polyline3d)
        endindex,end_point=self.get_index_and_z_from_x_on_polyline(endx,polyline3d)
        barplacementlist=[]
        leftbarcount=barcount
        newoffset=startx-polyline3d[startindex].X
        for i in range(startindex,endindex+1):
                line3d=polyline3d.GetLine(i)
                vector3d=self.line3d_to_vector3d(line3d)
                xdz=vector3d.Z/vector3d.X
                start_point=line3d.GetStartPoint()+AllplanGeo.Vector3D(newoffset,0,newoffset*xdz)
                distan=xdistance/vector3d.X*vector3d.GetLength()
                vector3d.Normalize(distan)
                position_point=start_point+vector3d
                newbarcount=int((polyline3d[i+1].X-polyline3d[i].X-newoffset)/xdistance)+1
                if(newbarcount>leftbarcount or i==endindex):
                    newbarcount=leftbarcount
                enddx=(newbarcount-1)*xdistance+start_point.X
                barplacement=self.barplacement_on_line(positionNumber,shape,newoffset,xdistance,newbarcount,line3d)
                barplacementlist.append(barplacement)
                newoffset=enddx+xdistance-polyline3d[i+1].X
                leftbarcount-=newbarcount
        return barplacementlist

    def get_index_and_z_from_x_on_polyline(self,x,polyline3d):
        '''
        x:x轴坐标
        polyline3d：3D折线
        根据x轴坐标和在x轴单向延伸的3D折线获取该坐标所对应的点及点所在的折线段数
        '''
        xlist=[]
        for i in range(0,polyline3d.Count()):
            xlist.append(polyline3d[i].X)
        index=0
        for i in range(0,len(xlist)-1):
            if((x-xlist[i])*(x-xlist[i+1])<=0):
                break
            index+=1
        if(index==polyline3d.Count()-1):
            if((x-polyline3d[0].X)*(polyline3d[1].X-polyline3d[0].X)>0):
                index=polyline3d.Count()-2
            else:
                index=0
        offset=x-xlist[index]
        line=polyline3d.GetLine(index)
        point=polyline3d[index]
        y=point.Y
        z=point.Z
        vector=self.line3d_to_vector3d(line)
        y+=offset/vector.X*vector.Y
        z+=offset/vector.X*vector.Z
        return index,AllplanGeo.Point3D(x,y,z)

    def div_barplacement_on_polyline(self,positionNumber, shape,offsetlist,xdistancelist,barcountlist,polyline3d):
        '''
        positionNumber：钢筋编号
        shape:钢筋形状
        offsetlist：起始偏移列表
        xdistance列表：钢筋间距在x轴投影长度列表
        barcount：钢筋条数列表
        polyline3d：所沿铺设折线
        生成同一种钢筋沿折线按不同区域不同间距线性铺设效果
        '''
        newbarplacementlist=[]
        totaloffset=0
        for j in range(0,len(offsetlist)):
            offset=offsetlist[j]
            totaloffset+=offset
            divdistan=xdistancelist[j]*(barcountlist[j]-1)
            barplace=self.barplacement_on_polyline(positionNumber,shape,totaloffset,xdistancelist[j],barcountlist[j],polyline3d)
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

    def polyline3d_reverse_longthen(self,polyline3d):
        '''
        polyline3d关于梁跨中心反向延伸 
        '''
        matmg=AllplanGeo.Matrix3D()
        matmg.Reflection(AllplanGeo.Plane3D(AllplanGeo.Point3D(self.halflength, 0, 0),AllplanGeo.Point3D(self.halflength, 0, 1),AllplanGeo.Point3D(self.halflength, 1, 0)))
        polyline=AllplanGeo.Polyline3D(polyline3d)
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

    def get_polyline_from_point(self,point):
        '''
        生成该点沿梁跨方向延伸梁长的折线
        '''
        polyline=AllplanGeo.Polyline3D()
        endpoint=point+AllplanGeo.Vector3D(2*self.halflength,0,0)
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
        求夹在两线段中间的线段两头延伸一段距离后与两线段的两交点的中点
        '''
        poly1=self.get_polyline_from_lines(line1,line)
        poly2=self.get_polyline_from_lines(line,line2)
        newline=AllplanGeo.Line3D(poly1[1],poly2[1])
        middle=newline.GetCenterPoint()
        return middle