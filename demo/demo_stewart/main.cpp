﻿#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

const char xml_data[] =
"<?xml version=\"1.0\" encoding=\"utf-8\"?>"
"<root>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"        <variable_pool type=\"VariablePoolObject\" default_child_type=\"Matrix\">"
"            <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"            <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"        </variable_pool>"
"        <akima_pool type=\"AkimaPoolObject\" default_child_type=\"Akima\">"
"        </akima_pool>"
"        <part_pool type=\"PartPoolObject\" default_child_type=\"Part\">"
"            <p1a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.999999999999974 , 1.22522177619812e-16 , -9.28869564848867e-18 , 6.38378239159465e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pa.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <u1i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p1j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"            </p1a>"
"            <p1b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.0711481425892889 , 1.49999999999963 , 0.912443796234424 , 8.04911692853238e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pb.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <p1i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </p1b>"
"            <p2a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.999999999999995 , 1.22524189323061e-16 , -9.2876368573046e-18 , 5.55111512312578e-17 , 0.721024145526766 , 0.308719565228027}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pa.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <u2i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p2j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"            </p2a>"
"            <p2b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.363127053316677 , 1.49999999999988 , 1.31832224563822 , 6.28318530717959 , 0.721024145526766 , 0.308719565228028}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pb.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <p2i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s2j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </p2b>"
"            <p3a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1.24902578429613e-16 , 3.6066466064807e-14 , 1.73199999999999 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pa.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <u3i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p3j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"            </p3a>"
"            <p3b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.363127053316337 , 1.49999999999935 , 1.31832224563851 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pb.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <p3i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s3j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </p3b>"
"            <p4a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1.24898250620648e-16 , 1.52855080404276e-14 , 1.732 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pa.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <u4i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p4j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"            </p4a>"
"            <p4b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.134375029322252 , 1.49999999999964 , 1.36823895396183 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pb.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <p4i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s4j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </p4b>"
"            <p5a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.999999999999993 , -1.0082029353865e-16 , 4.19175032725778e-17 , 6.28318530717959 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pa.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <u5i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p5j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"            </p5a>"
"            <p5b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.134375029322406 , 1.49999999999987 , 1.36823895396163 , 2.77555756156289e-17 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pb.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <p5i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s5j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </p5b>"
"            <p6a active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{-0.999999999999969 , -1.00821934664985e-16 , 4.19165900651815e-17 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pa.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <u6i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                    <p6j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                </marker_pool>"
"            </p6a>"
"            <p6b active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.0711481425888235 , 1.49999999999959 , 0.912443796234401 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\pb.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <p6i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                    <s6j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </p6b>"
"            <up active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0.1 , 1.5 , 1.2 , 1.5707963267949 , 0.1 , 4.71238898038469}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\up.xmt_txt\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <ee pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <s1i pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                    <s2i pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                    <s3i pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                    <s4i pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                    <s5i pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                    <s6i pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                </marker_pool>"
"            </up>"
"            <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <ref pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <u1o pe=\"{ 1,0,0,0,0,0 }\"/>"
"                    <u2o pe=\"{ 1,0,0,0,0,0 }\"/>"
"                    <u3o pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                    <u4o pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                    <u5o pe=\"{ -1,0,0,0,0,0 }\"/>"
"                    <u6o pe=\"{ -1,0,0,0,0,0 }\"/>"
"                    <u1j pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                    <u2j pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                    <u3j pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                    <u4j pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                    <u5j pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                    <u6j pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                </marker_pool>"
"            </ground>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolObject\">"
"            <u1 active=\"true\" type=\"UniversalJoint\" prt_m=\"p1a\" prt_n=\"ground\" mak_i=\"u1i\" mak_j=\"u1j\"/>"
"            <p1 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\"/>"
"            <s1 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p1b\" mak_i=\"s1i\" mak_j=\"s1j\"/>"
"            <u2 active=\"true\" type=\"UniversalJoint\" prt_m=\"p2a\" prt_n=\"ground\" mak_i=\"u2i\" mak_j=\"u2j\"/>"
"            <p2 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\"/>"
"            <s2 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p2b\" mak_i=\"s2i\" mak_j=\"s2j\"/>"
"            <u3 active=\"true\" type=\"UniversalJoint\" prt_m=\"p3a\" prt_n=\"ground\" mak_i=\"u3i\" mak_j=\"u3j\"/>"
"            <p3 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\"/>"
"            <s3 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p3b\" mak_i=\"s3i\" mak_j=\"s3j\"/>"
"            <u4 active=\"true\" type=\"UniversalJoint\" prt_m=\"p4a\" prt_n=\"ground\" mak_i=\"u4i\" mak_j=\"u4j\"/>"
"            <p4 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\"/>"
"            <s4 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p4b\" mak_i=\"s4i\" mak_j=\"s4j\"/>"
"            <u5 active=\"true\" type=\"UniversalJoint\" prt_m=\"p5a\" prt_n=\"ground\" mak_i=\"u5i\" mak_j=\"u5j\"/>"
"            <p5 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\"/>"
"            <s5 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p5b\" mak_i=\"s5i\" mak_j=\"s5j\"/>"
"            <u6 active=\"true\" type=\"UniversalJoint\" prt_m=\"p6a\" prt_n=\"ground\" mak_i=\"u6i\" mak_j=\"u6j\"/>"
"            <p6 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\"/>"
"            <s6 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p6b\" mak_i=\"s6i\" mak_j=\"s6j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolObject\" default_child_type=\"Motion\">"
"            <m1 active=\"true\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m2 active=\"true\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m3 active=\"true\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m4 active=\"true\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m5 active=\"true\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"            <m6 active=\"true\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolObject\" default_child_type=\"GeneralMotion\">"
"            <general_motion active=\"true\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"ref\"/>"
"        </general_motion_pool>"
"    </model>"
"</root>";


//class Robot :public aris::dynamic::Model
//{
//public:
//	using Model::loadXml;
//	auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
//	{
//		Model::loadXml(xml_ele);
//
//		up_ = &*partPool().findByName("up");
//		p1a_ = &*partPool().findByName("p1a");
//		p1b_ = &*partPool().findByName("p1b");
//		p2a_ = &*partPool().findByName("p2a");
//		p2b_ = &*partPool().findByName("p2b");
//		p3a_ = &*partPool().findByName("p3a");
//		p3b_ = &*partPool().findByName("p3b");
//		p4a_ = &*partPool().findByName("p4a");
//		p4b_ = &*partPool().findByName("p4b");
//		p5a_ = &*partPool().findByName("p5a");
//		p5b_ = &*partPool().findByName("p5b");
//		p6a_ = &*partPool().findByName("p6a");
//		p6b_ = &*partPool().findByName("p6b");
//
//		m1_ = &*motionPool().findByName("m1");
//		m2_ = &*motionPool().findByName("m2");
//		m3_ = &*motionPool().findByName("m3");
//		m4_ = &*motionPool().findByName("m4");
//		m5_ = &*motionPool().findByName("m5");
//		m6_ = &*motionPool().findByName("m6");
//	}
//
//	auto virtual kinFromPin()->void override {};
//	auto virtual kinFromVin()->void override {};
//
//	auto setPee(double *pee)
//	{
//		up_->setPe(pee, "321");
//
//		double pp[3], pe[6]{ 0,0,0,0,0,0 }, pe2[6]{ 0,0,0,0,0,0 };
//		up_->markerPool().findByName("s1i")->getPp(*ground().markerPool().findByName("u1o"), pp);
//		s_sov_ab(pp, pe + 3, "132");
//		p1a_->setPe(*ground().markerPool().findByName("u1o"), pe, "132");
//		pe2[1] = s_norm(3, pp, 1);
//		p1b_->setPe(*p1a_, pe2);
//
//		up_->markerPool().findByName("s2i")->getPp(*ground().markerPool().findByName("u2o"), pp);
//		s_sov_ab(pp, pe + 3, "132");
//		p2a_->setPe(*ground().markerPool().findByName("u2o"), pe, "132");
//		pe2[1] = s_norm(3, pp, 1);
//		p2b_->setPe(*p2a_, pe2);
//
//		up_->markerPool().findByName("s3i")->getPp(*ground().markerPool().findByName("u3o"), pp);
//		s_sov_ab(pp, pe + 3, "132");
//		p3a_->setPe(*ground().markerPool().findByName("u3o"), pe, "132");
//		pe2[1] = s_norm(3, pp, 1);
//		p3b_->setPe(*p3a_, pe2);
//
//		up_->markerPool().findByName("s4i")->getPp(*ground().markerPool().findByName("u4o"), pp);
//		s_sov_ab(pp, pe + 3, "132");
//		p4a_->setPe(*ground().markerPool().findByName("u4o"), pe, "132");
//		pe2[1] = s_norm(3, pp, 1);
//		p4b_->setPe(*p4a_, pe2);
//
//		up_->markerPool().findByName("s5i")->getPp(*ground().markerPool().findByName("u5o"), pp);
//		s_sov_ab(pp, pe + 3, "132");
//		p5a_->setPe(*ground().markerPool().findByName("u5o"), pe, "132");
//		pe2[1] = s_norm(3, pp, 1);
//		p5b_->setPe(*p5a_, pe2);
//
//		up_->markerPool().findByName("s6i")->getPp(*ground().markerPool().findByName("u6o"), pp);
//		s_sov_ab(pp, pe + 3, "132");
//		p6a_->setPe(*ground().markerPool().findByName("u6o"), pe, "132");
//		pe2[1] = s_norm(3, pp, 1);
//		p6b_->setPe(*p6a_, pe2);
//
//		for (auto &mot : motionPool())
//		{
//			mot.updMp();
//			mot.updMv();
//		}
//	};
//	auto setVee(double *vee, double *pee)
//	{
//		up_->setVe(vee, pee, "321");
//
//		double pe[6]{ 0,0,0,0,0,0 }, pe2[6]{ 0,0,0,0,0,0 }, ve[6]{ 0,0,0,0,0,0 }, ve2[6]{ 0,0,0,0,0,0 };
//		up_->markerPool().findByName("s1i")->getPp(*ground().markerPool().findByName("u1o"), pe2);
//		up_->markerPool().findByName("s1i")->getVp(*ground().markerPool().findByName("u1o"), ve2);
//		s_sov_vab(pe2, ve2, ve + 3, pe + 3, "132");
//		s_sov_vab(pe2, ve2, ve2 + 3, pe2 + 3, "132");
//		p1a_->setVe(*ground().markerPool().findByName("u1o"), ve, pe, "132");
//		p1b_->setVe(*ground().markerPool().findByName("u1o"), ve2, pe2, "132");
//
//		up_->markerPool().findByName("s2i")->getPp(*ground().markerPool().findByName("u2o"), pe2);
//		up_->markerPool().findByName("s2i")->getVp(*ground().markerPool().findByName("u2o"), ve2);
//		s_sov_vab(pe2, ve2, ve + 3, pe + 3, "132");
//		s_sov_vab(pe2, ve2, ve2 + 3, pe2 + 3, "132");
//		p2a_->setVe(*ground().markerPool().findByName("u2o"), ve, pe, "132");
//		p2b_->setVe(*ground().markerPool().findByName("u2o"), ve2, pe2, "132");
//
//		up_->markerPool().findByName("s3i")->getPp(*ground().markerPool().findByName("u3o"), pe2);
//		up_->markerPool().findByName("s3i")->getVp(*ground().markerPool().findByName("u3o"), ve2);
//		s_sov_vab(pe2, ve2, ve + 3, pe + 3, "132");
//		s_sov_vab(pe2, ve2, ve2 + 3, pe2 + 3, "132");
//		p3a_->setVe(*ground().markerPool().findByName("u3o"), ve, pe, "132");
//		p3b_->setVe(*ground().markerPool().findByName("u3o"), ve2, pe2, "132");
//
//		up_->markerPool().findByName("s4i")->getPp(*ground().markerPool().findByName("u4o"), pe2);
//		up_->markerPool().findByName("s4i")->getVp(*ground().markerPool().findByName("u4o"), ve2);
//		s_sov_vab(pe2, ve2, ve + 3, pe + 3, "132");
//		s_sov_vab(pe2, ve2, ve2 + 3, pe2 + 3, "132");
//		p4a_->setVe(*ground().markerPool().findByName("u4o"), ve, pe, "132");
//		p4b_->setVe(*ground().markerPool().findByName("u4o"), ve2, pe2, "132");
//
//		up_->markerPool().findByName("s5i")->getPp(*ground().markerPool().findByName("u5o"), pe2);
//		up_->markerPool().findByName("s5i")->getVp(*ground().markerPool().findByName("u5o"), ve2);
//		s_sov_vab(pe2, ve2, ve + 3, pe + 3, "132");
//		s_sov_vab(pe2, ve2, ve2 + 3, pe2 + 3, "132");
//		p5a_->setVe(*ground().markerPool().findByName("u5o"), ve, pe, "132");
//		p5b_->setVe(*ground().markerPool().findByName("u5o"), ve2, pe2, "132");
//
//		up_->markerPool().findByName("s6i")->getPp(*ground().markerPool().findByName("u6o"), pe2);
//		up_->markerPool().findByName("s6i")->getVp(*ground().markerPool().findByName("u6o"), ve2);
//		s_sov_vab(pe2, ve2, ve + 3, pe + 3, "132");
//		s_sov_vab(pe2, ve2, ve2 + 3, pe2 + 3, "132");
//		p6a_->setVe(*ground().markerPool().findByName("u6o"), ve, pe, "132");
//		p6b_->setVe(*ground().markerPool().findByName("u6o"), ve2, pe2, "132");
//
//		for (auto &mot : motionPool())
//		{
//			mot.updMp();
//			mot.updMv();
//		}
//	};
//
//	auto p1a()->Part& { return *p1a_; }
//	auto p1b()->Part& { return *p1b_; }
//	auto p2a()->Part& { return *p2a_; }
//	auto p2b()->Part& { return *p2b_; }
//	auto p3a()->Part& { return *p3a_; }
//	auto p3b()->Part& { return *p3b_; }
//	auto p4a()->Part& { return *p4a_; }
//	auto p4b()->Part& { return *p4b_; }
//	auto p5a()->Part& { return *p5a_; }
//	auto p5b()->Part& { return *p5b_; }
//	auto p6a()->Part& { return *p6a_; }
//	auto p6b()->Part& { return *p6b_; }
//	auto up()->Part& { return *up_; }
//
//	auto m1()->Motion& { return *m1_; }
//	auto m2()->Motion& { return *m2_; }
//	auto m3()->Motion& { return *m3_; }
//	auto m4()->Motion& { return *m4_; }
//	auto m5()->Motion& { return *m5_; }
//	auto m6()->Motion& { return *m6_; }
//
//private:
//	aris::dynamic::Part *p1a_, *p1b_, *p2a_, *p2b_, *p3a_, *p3b_, *p4a_, *p4b_, *p5a_, *p5b_, *p6a_, *p6b_, *up_;
//	aris::dynamic::Motion *m1_, *m2_, *m3_, *m4_, *m5_, *m6_;
//	//aris::dynamic::Joint *u1, *p1, *s1, *p5b_, *p5a_, *p5b_, *p5a_, *p5b_, *p5a_, *p5b_, *p6a_, *p6b_;
//};


int main()
{
	std::cout << std::endl << "-----------------test model stewart---------------------" << std::endl;

	//try
	//{
	//	Model rbt;
	//	rbt.loadString(xml_file);

	//	rbt.generalMotionPool().findByName("general_motion")->activate(true);
	//	for (auto &m : rbt.motionPool())m.activate(false);

	//	double pee[6]{ 0.0 , 1.5 , 1.2 , 0.0 , 0.1 , 0.0 };
	//	rbt.generalMotionPool().findByName("general_motion")->setMp(pee);

	//	rbt.allocateMemory();
	//	auto ret = rbt.kinPos(100);

	//	std::cout << "computation finished, spend " << std::get<0>(ret) << " count with error " << std::get<1>(ret) << std::endl;

	//	rbt.saveAdams("C:\\Users\\py033\\Desktop\\stewart.cmd");
	//}
	//catch (std::exception&e)
	//{
	//	std::cout << e.what() << std::endl;
	//}

	//try
	//{
	//	Model rbt;
	//	rbt.loadString(xml_file);

	//	rbt.generalMotionPool().findByName("general_motion")->activate(false);
	//	for (auto &m : rbt.motionPool())m.activate(true);

	//	double pin[6]{ 2.0,2.0,2.0,2.0,2.0,2.0 };
	//	for (int i{ 0 }; i < 6; ++i) rbt.motionAtAbs(i).setMp(pin[i]);

	//	rbt.allocateMemory();
	//	auto ret = rbt.kinPos(100);
	//	std::cout << "computation finished, spend " << std::get<0>(ret) << " count with error " << std::get<1>(ret) << std::endl;

	//	rbt.saveAdams("C:\\Users\\py033\\Desktop\\stewart.cmd");
	//}
	//catch (std::exception&e)
	//{
	//	std::cout << e.what() << std::endl;
	//}


	//// test inverse efficiency
	//try
	//{
	//	Model rbt;
	//	rbt.loadString(xml_file);

	//	auto gmt = rbt.generalMotionPool().findByName("general_motion");
	//	gmt->activate(true);
	//	for (auto &m : rbt.motionPool())m.activate(false);

	//	
	//	rbt.allocateMemory();

	//	std::cout << "inverse time:" << aris::core::benchmark(1000, [&]()
	//	{
	//		double pee1[6]{ 0.0 , 1.5 , 1.2 , 0.0 , 0.1 , 0.0 };
	//		double pee2[6]{ 0.0 , 1.6 , 1.25 , 0.11 , 0.12 , 0.06 };

	//		gmt->setMp(pee1);
	//		auto ret1 = rbt.kinPos(100);

	//		gmt->setMp(pee2);
	//		auto ret2 = rbt.kinPos(100);
	//	}) << std::endl;
	//}
	//catch (std::exception&e)
	//{
	//	std::cout << e.what() << std::endl;
	//}

	// test forward efficiency
	//try
	//{
	//	Model rbt;
	//	rbt.loadXmlStr(xml_data);

	//	rbt.generalMotionPool().findByName("general_motion")->activate(false);
	//	for (auto &m : rbt.motionPool())m.activate(true);

	//	//rbt.allocateMemory();

	//	std::cout << "forward time:" << aris::core::benchmark(1000, [&]()
	//	{
	//		double pin1[6]{ 2.0 , 2.0 , 2.0 , 2.0 , 2.0 , 2.0 };
	//		double pin2[6]{ 2.1 , 1.98 , 2.05 , 2.13 , 1.86 , 1.88 };

	//		for (int i{ 0 }; i < 6; ++i) rbt.motionPool().at(i).setMp(pin1[i]);
	//		//auto ret1 = rbt.kinPosInGlb(100);

	//		for (int i{ 0 }; i < 6; ++i) rbt.motionPool().at(i).setMp(pin2[i]);
	//		//auto ret2 = rbt.kinPosInGlb(100);
	//	}) << std::endl;
	//}
	//catch (std::exception&e)
	//{
	//	std::cout << e.what() << std::endl;
	//}


	try
	{
		double a[6][3]
		{
			0.45, - 0.433012701892219  ,                       0,
			0.6, - 0.173205080756888     ,                    0,
			0.15,         0.606217782649107       ,                  0,
			- 0.15,         0.606217782649107     ,                    0,
			- 0.6, - 0.173205080756888            ,             0,
			- 0.45, - 0.433012701892219           ,              0,
		};
		double b[6][3]
		{
			0.125, - 0.476313972081441          ,             0.8,
			0.475,         0.129903810567666     ,                  0.8,
			0.35   ,      0.346410161513775        ,               0.8,
			- 0.35 ,        0.346410161513775     ,                  0.8,
			- 0.475 ,        0.129903810567666     ,                  0.8,
			- 0.125, - 0.476313972081441           ,            0.8,
		};
		
		double R1a_pos[3]{ 0.45, -0.433012701892219,                         0 };
		double R1a_axis[3]{ 0.5     ,    0.866025403784439                    ,     0 };
		double R1b_pos[3]{ 0.45, -0.433012701892219                     ,    0 };
		double R1b_axis[3]{ 0.823677867665496, -0.475550638622211      ,   0.308879200374561 };
		double P1_pos[3]{ 0.45, -0.433012701892219                       ,  0 };
		double P1_axis[3]{ -0.375904705778056, -0.0500835422470634     ,    0.925303891145984 };
		double S1_pos[3]{ 0.125, -0.476313972081441               ,        0.8 };
		double R2a_pos[3]{ 0.6, -0.173205080756888        ,                 0 };
		double R2a_axis[3]{ 0.5,         0.866025403784439          ,               0 };
		double R2b_pos[3]{ 0.6, -0.173205080756888                     ,    0 };
		double R2b_axis[3]{ 0.823677867665496, -0.475550638622211    ,     0.308879200374561 };
		double P2_pos[3]{ 0.6, -0.173205080756888                   ,      0 };
		double P2_axis[3]{ -0.14457873299156,         0.350584795729443      ,   0.925303891145984 };
		double S2_pos[3]{ 0.475,         0.129903810567666               ,        0.8 };
		double R3a_pos[3]{ 0.15,         0.606217782649107              ,           0 };
		double R3a_axis[3]{ -1,     0    , 0 };
		double R3b_pos[3]{ 0.15,         0.606217782649107 ,                        0 };
		double R3b_axis[3]{ 0  ,       0.951101277244422    ,     0.308879200374561 };
		double P3_pos[3]{ 0.15 ,        0.606217782649107   ,                      0 };
		double P3_axis[3]{ 0.231325972786496, -0.30050125348238  ,       0.925303891145984 };
		double S3_pos[3]{ 0.35  ,       0.346410161513775   ,                    0.8 };
		double R4a_pos[3]{ -0.15  ,       0.606217782649107    ,                     0 };
		double R4a_axis[3]{ -1  ,   0  ,   0 };
		double R4b_pos[3]{ -0.15 ,        0.606217782649107  ,                       0 };
		double R4b_axis[3]{ 0    ,     0.951101277244422     ,    0.308879200374561 };
		double P4_pos[3]{ -0.15   ,      0.606217782649107      ,                   0 };
		double P4_axis[3]{ -0.231325972786496, -0.30050125348238     ,    0.925303891145984 };
		double S4_pos[3]{ -0.35   ,      0.346410161513775         ,              0.8 };
		double R5a_pos[3]{ -0.6, -0.173205080756888                ,         0 };
		double R5a_axis[3]{ 0.5, -0.866025403784439               ,          0 };
		double R5b_pos[3]{ -0.6, -0.173205080756888               ,          0 };
		double R5b_axis[3]{ -0.823677867665496, -0.475550638622211  ,       0.308879200374561 };
		double P5_pos[3]{ -0.6, -0.173205080756888                   ,      0 };
		double P5_axis[3]{ 0.14457873299156,         0.350584795729443 ,        0.925303891145984 };
		double S5_pos[3]{ -0.475,         0.129903810567666          ,             0.8 };
		double R6a_pos[3]{ -0.45, -0.433012701892219                ,         0 };
		double R6a_axis[3]{ 0.5, -0.866025403784439                ,         0 };
		double R6b_pos[3]{ -0.45, -0.433012701892219                ,         0 };
		double R6b_axis[3]{ -0.823677867665496, -0.475550638622211   ,      0.308879200374561 };
		double P6_pos[3]{ -0.45, -0.433012701892219                    ,     0 };
		double P6_axis[3]{ 0.375904705778056, -0.0500835422470634      ,   0.925303891145984 };
		double S6_pos[3]{ -0.125, -0.476313972081441                   ,    0.8 };



		Model rbt;
		s_vc(6, std::array<double, 6>{0.0, 0.0, -9.8, 0.0, 0.0, 0.0}.data(), const_cast<double *>(rbt.environment().gravity()));

		auto &up = rbt.partPool().add<Part>("up");

		auto &p1a = rbt.partPool().add<Part>("p1a");
		auto &p1b = rbt.partPool().add<Part>("p1b");
		auto &p1c = rbt.partPool().add<Part>("p1c");

		auto &p2a = rbt.partPool().add<Part>("p2a");
		auto &p2b = rbt.partPool().add<Part>("p2b");
		auto &p2c = rbt.partPool().add<Part>("p2c");

		auto &p3a = rbt.partPool().add<Part>("p3a");
		auto &p3b = rbt.partPool().add<Part>("p3b");
		auto &p3c = rbt.partPool().add<Part>("p3c");

		auto &p4a = rbt.partPool().add<Part>("p4a");
		auto &p4b = rbt.partPool().add<Part>("p4b");
		auto &p4c = rbt.partPool().add<Part>("p4c");

		auto &p5a = rbt.partPool().add<Part>("p5a");
		auto &p5b = rbt.partPool().add<Part>("p5b");
		auto &p5c = rbt.partPool().add<Part>("p5c");

		auto &p6a = rbt.partPool().add<Part>("p6a");
		auto &p6b = rbt.partPool().add<Part>("p6b");
		auto &p6c = rbt.partPool().add<Part>("p6c");

		auto &r1a = rbt.addRevoluteJoint(rbt.ground(), p1a, R1a_pos, R1a_axis);
		auto &r1b = rbt.addRevoluteJoint(p1a, p1b, R1b_pos, R1b_axis);
		auto &p1 = rbt.addPrismaticJoint(p1c, p1b, P1_pos, P1_axis);
		auto &s1 = rbt.addSphericalJoint(p1c, up, S1_pos);

		auto &r2a = rbt.addRevoluteJoint(rbt.ground(), p2a, R2a_pos, R2a_axis);
		auto &r2b = rbt.addRevoluteJoint(p2a, p2b, R2b_pos, R2b_axis);
		auto &p2 = rbt.addPrismaticJoint(p2c, p2b, P2_pos, P2_axis);
		auto &s2 = rbt.addSphericalJoint(p2c, up, S2_pos);
		
		auto &r3a = rbt.addRevoluteJoint(rbt.ground(), p3a, R3a_pos, R3a_axis);
		auto &r3b = rbt.addRevoluteJoint(p3a, p3b, R3b_pos, R3b_axis);
		auto &p3 = rbt.addPrismaticJoint(p3c, p3b, P3_pos, P3_axis);
		auto &s3 = rbt.addSphericalJoint(p3c, up, S3_pos);

		auto &r4a = rbt.addRevoluteJoint(rbt.ground(), p4a, R4a_pos, R4a_axis);
		auto &r4b = rbt.addRevoluteJoint(p4a, p4b, R4b_pos, R4b_axis);
		auto &p4 = rbt.addPrismaticJoint(p4c, p4b, P4_pos, P4_axis);
		auto &s4 = rbt.addSphericalJoint(p4c, up, S4_pos);

		auto &r5a = rbt.addRevoluteJoint(rbt.ground(), p5a, R5a_pos, R5a_axis);
		auto &r5b = rbt.addRevoluteJoint(p5a, p5b, R5b_pos, R5b_axis);
		auto &p5 = rbt.addPrismaticJoint(p5c, p5b, P5_pos, P5_axis);
		auto &s5 = rbt.addSphericalJoint(p5c, up, S5_pos);

		auto &r6a = rbt.addRevoluteJoint(rbt.ground(), p6a, R6a_pos, R6a_axis);
		auto &r6b = rbt.addRevoluteJoint(p6a, p6b, R6b_pos, R6b_axis);
		auto &p6 = rbt.addPrismaticJoint(p6c, p6b, P6_pos, P6_axis);
		auto &s6 = rbt.addSphericalJoint(p6c, up, S6_pos);

		auto &m1 = rbt.addMotion(p1);
		auto &m2 = rbt.addMotion(p2);
		auto &m3 = rbt.addMotion(p3);
		auto &m4 = rbt.addMotion(p4);
		auto &m5 = rbt.addMotion(p5);
		auto &m6 = rbt.addMotion(p6);


		rbt.ground().geometryPool().add<ParasolidGeometry>("ground","C:\\aris\\resource\\demo_stewart\\ground.xmt_txt");
		up.geometryPool().add<ParasolidGeometry>("up", "C:\\aris\\resource\\demo_stewart\\up.xmt_txt");


		double p1b_pm[16]{ 0.823677867665496,         0.424559091872891, -0.375904705778056,                      0.45,
			-0.475550638622211,         0.878261480939471, -0.0500835422470634, -0.433012701892219,
			0.308879200374561,         0.220014428177045 ,        0.925303891145984    ,                     0,
			0  ,                       0 ,                        0                ,         1 };

		double p1c_pm[16]{0.823677867665496, 0.424559091872891, -0.375904705778056, 0.125,
        -0.475550638622211, 0.878261480939471, -0.0500835422470634, -0.476313972081441,
		0.308879200374561,0.220014428177045,0.925303891145984,   0.8,
	    0,     0,     0,     1 };

		double p2b_pm[16]{ 0.823677867665496,0.548317207722479,-0.14457873299156,0.6,
		-0.475550638622211,0.806809699439311,0.350584795729443,-0.173205080756888,
		 0.308879200374561,-0.220014428177045,0.925303891145984,0,
						 0,0,0,1 };


		double p2c_pm[16]{
0.823677867665496,0.548317207722479,-0.14457873299156,0.475,
-0.475550638622211,0.806809699439311,0.350584795729443,0.129903810567666,
 0.308879200374561,-0.220014428177045,0.925303891145984,0.8,
				 0,0,0,1 };
		double p3b_pm[16]{ 0,-0.972876299595371,0.231325972786496,0.15,
				 0.951101277244422,-0.0714517815001604,-0.30050125348238,0.606217782649107,
				 0.308879200374561,0.220014428177045,0.925303891145984,0,
								 0,0,0,1 };


		double p3c_pm[16]{

						 0,-0.972876299595371,0.231325972786496,0.35,
		 0.951101277244422,-0.0714517815001604,-0.30050125348238,0.346410161513775,
		 0.308879200374561,0.220014428177045,0.925303891145984,0.8,
						 0,0,0,1 };


			double p4b_pm [16]{

                         0,-0.972876299595371,-0.231325972786496,-0.15,
         0.951101277244422,0.0714517815001604,-0.30050125348238,0.606217782649107,
         0.308879200374561,-0.220014428177045,0.925303891145984,0,
                         0,0,0,1};


			double p4c_pm[16]{

						 0,-0.972876299595371,-0.231325972786496,-0.35,
		 0.951101277244422,0.0714517815001604,-0.30050125348238,0.346410161513775,
		 0.308879200374561,-0.220014428177045,0.925303891145984,0.8,
						 0,0,0,1 };


			double p5b_pm[16]{

		-0.823677867665496,0.548317207722479,0.14457873299156,-0.6,
		-0.475550638622211,-0.806809699439311,0.350584795729443,-0.173205080756888,
		 0.308879200374561,0.220014428177045,0.925303891145984,0,
						 0,0,0,1 };


			double p5c_pm[16]{

		-0.823677867665496,0.548317207722479,0.14457873299156,-0.475,
		-0.475550638622211,-0.806809699439311,0.350584795729443,0.129903810567666,
		 0.308879200374561,0.220014428177045,0.925303891145984,0.8,
						 0,0,0,1 };


			double p6b_pm[16]{

		-0.823677867665496,0.424559091872891,0.375904705778056,-0.45,
		-0.475550638622211,-0.878261480939471,-0.0500835422470634,-0.433012701892219,
		 0.308879200374561,-0.220014428177045,0.925303891145984,0,
						 0,0,0,1 };


			double p6c_pm[16]{

		-0.823677867665496,0.424559091872891,0.375904705778056,-0.125,
		-0.475550638622211,-0.878261480939471,-0.0500835422470634,-0.476313972081441,
		 0.308879200374561,-0.220014428177045,0.925303891145984,0.8,
						 0,0,0,1 };

		p1b.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1b.xmt_txt", p1b_pm);
		p1c.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1c.xmt_txt", p1c_pm);

		p2b.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1b.xmt_txt", p2b_pm);
		p2c.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1c.xmt_txt", p2c_pm);

		p3b.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1b.xmt_txt", p3b_pm);
		p3c.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1c.xmt_txt", p3c_pm);

		p4b.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1b.xmt_txt", p4b_pm);
		p4c.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1c.xmt_txt", p4c_pm);

		p5b.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1b.xmt_txt", p5b_pm);
		p5c.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1c.xmt_txt", p5c_pm);

		p6b.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1b.xmt_txt", p6b_pm);
		p6c.geometryPool().add<ParasolidGeometry>("ground", "C:\\aris\\resource\\demo_stewart\\p1c.xmt_txt", p6c_pm);


		auto &ee = rbt.addGeneralMotionByPe(up, rbt.ground(), nullptr, "321");

		ee.setMpe(std::array<double, 6>{0.01, 0.02, 0.03, 0.3,0.2,0.1}.data(), "321");
		ee.setMvs(std::array<double, 6>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}.data());

		auto &s = rbt.solverPool().add<UniversalSolver>();
		s.setMaxError(1e-14);
		//double theta[6]{ 0.1, 0.2, 0.3, 0.5, 0.6, 0.6 };
		//double theta_dot[6]{ 0.1, 0.2, 0.4, 0.35, 0.5, 0.78 };
		//double theta_dot_dot[6]{ 0.5,0.2,0.3,0.4,0.5,0.6 };

		//for (aris::Size i = 0; i < 6; ++i)
		//{
		//	rbt.motionPool().at(i).setMp(theta[i]);
		//	rbt.motionPool().at(i).setMv(theta_dot[i]);
		//	rbt.motionPool().at(i).setMa(theta_dot_dot[i]);
		//}

		for (auto &m : rbt.motionPool())m.activate(false);

		s.allocateMemory();
		s.kinPos();

		for (auto &m : rbt.motionPool())m.activate(true);
		ee.activate(false);


		for (aris::Size i = 0; i < 6; ++i)
		{
			rbt.motionPool().at(i).setMv(i*0.1 + 0.1);
			rbt.motionPool().at(i).setMa(i*0.1 + 0.1);
		}

		s.allocateMemory();
		s.kinVel();
		s.dynAccAndFce();

		dsp(4, 4, *p5a.pm());
		dsp(4, 4, *p5b.pm());
		dsp(4, 4, *p5c.pm());


		for (aris::Size i = 0; i < 6; ++i)
		{
			rbt.motionPool().at(i).updMp();
			std::cout << std::setprecision(15) << rbt.motionPool().at(i).mv() << "  ";
		}
		std::cout << std::endl;
		for (aris::Size i = 0; i < 6; ++i)
		{
			std::cout << std::setprecision(15) << rbt.motionPool().at(i).ma() << "  ";
		}
		std::cout << std::endl;
		for (aris::Size i = 0; i < 6; ++i)
		{
			std::cout << std::setprecision(15) << rbt.motionPool().at(i).mf() << "  ";
		}
		std::cout << std::endl;

		ee.updMvs();
		ee.updMas();
		dsp(1, 6, ee.mvs());
		dsp(1, 6, ee.mas());

		auto &adams = rbt.simulatorPool().add<AdamsSimulator>();
		adams.saveAdams("C:\\Users\\py033\\Desktop\\stewart");

	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}









	std::cout << "demo_stewart finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

