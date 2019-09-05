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
        i=0
        for i in range(0,self.secnum):
            innersect,down,mid,up=self.create_common_inner_section_path(id1list[i],id2list[i],height,uplinenumlist[i],ie1list[i],
                in1list[i],ie2list[i],in2list[i],ie3list[i],in3list[i],downlinenumlist[i],if1list[i],im1list[i],if2list[i],im2list[i],
                if3list[i],im3list[i],ibo1list[i],icelist[i],ibo2list[i],icflist[i],secdislist[i])
            innersectionlist.append(innersect)
            downlist.append(down)
            midlist.append(mid)
            uplist.append(up)
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
        
        #是否显示区域块
        
        model_ele_list=[]
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
        #CF1倒角
        if bo2:
            err1,ln1,midline,fil0=AllplanGeo.FilletCalculus3D.Calculate(downlinelist[-1],midline,cf1)
            lpath+=ln1
            lpath+=fil0
            downlinelist[-1]=ln1
            downptlist[-1]=ln1.GetEndPoint()
        else:
            lpath+=downlinelist[-1]
        #CE1倒角
        if bo1:
            err2,midline,ln2,fil1=AllplanGeo.FilletCalculus3D.Calculate(midline,uplinelist[0],ce1)
            lpath+=midline
            lpath+=fil1
            lpath+=ln2
            uplinelist[0]=ln2
            upptlist[0]=ln2.GetStartPoint()
        else:
            lpath+=midline
            lpath+=uplinelist[0]
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
        return (lpath,down,mid,up)

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

        dislistn5_1=[50+22/2+18/2,100,125*2,125*2,125*2,125*2,100-22-18]
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
        #N16
        profile,hooklength=Createsteelshape.shape_N16_steel()
        shape_props = ReinforcementShapeProperties.rebar(16,r,self.StirSteelGrade,self.ConcreteGrade, AllplanReinf.BendingShapeType.Stirrup)
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
        reinforce.append(N16)
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
