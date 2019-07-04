/*
 * Copyright 2019 Nagoya University
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _DMP_DATA_H_
#define _DMP_DATA_H_

#include <string>
#include <vector>
#include <set>
#include <map>

/****************************************
*  定数定義
****************************************/

/****************************************
*  インデックス定義
****************************************/

// 地物カテゴリ(Feature category)定義コード
typedef enum
{
  CAT_NONE  = 0,   // なし
  CAT_Point = 1,   // 点地物
  CAT_Line  = 2,   // 線地物
  CAT_Area  = 3,   // 面地物
  CAT_Complex = 4  // 複合地物
} DmpFtrCat_e;

// 地物(Feature)定義コード
typedef enum
{
  F_NONE = 0,                    // 0: NONE
  FL4110_RoadElement,            // 1:リンクレベル地物
  FL4115_Pathway,                // 2:歩道中心線
  FP4120_Junction,               // 3:○ジャンクション(子)
  FC4140_Road,                   // 4:リンクレベル地物
  FC4145_Intersection,           // 5:交差点(親)
  FP7210_Signpost,               // 6:(使用しない)
  FP7220_TrafficSign,            // 7:標識(親／子)
  FP7230_TrafficLight,           // 8:(使用しない)
  FP7240_PedestrianCrossing,     // 9:(使用しない)
  FC7245_ComplexPedestrianCrossing,  // 10:
  FP7251_EnvironmentalEquipment,  //11:
  FP7252_Lighting,               // 12:街灯(子)
  FP7254_RoadMarkings,           // 13:路面マーク中心点(子)
  FC8110_Lane,                   // 14:○レーン本体(親)
  FA8120_LaneArea,               // 15:○レーン走行可能領域(子)
  FL8130_LaneLine,               // 16:○レーン中心線/走行目安線(子)
  FC8140_ExtendedPedestrian,     // 17:歩道(親)
  FC8145_ExtendedPedestrianCrossing, // 18:横断歩道(親)
  FA8150_PedestrianArea,         // 19:歩道／横断歩道(縞模様の外形)形状(子)
  FL8160_PedestrianLine,         // 20:歩道／横断歩道中心線(子)
  FA8170_IntersectionAreaShape,  // 21:交差点領域(親／子)
  FP8210_Pole,                   // 22:○ポール(親／子)
  FC8220_ExtendedTrafficSign,    // 23:○標識(親)
  FC8230_ExtendedTrafficLight,   // 24:○信号(親)
  FP8231_TrafficLightLamp,       // 25:○信号ランプ(子)
  FC8240_ExtendedLighting,       // 26:街灯(親)
  FL8241_LightingLamp,           // 27:街灯ランプ(子)
  FC8250_ExtendedRoadMarkings,   // 28:路面マーク(親)
  FA8251_RoadMarkingsShape,      // 29:路面マーク形状(子)
  FA8252_PedestrianCrossingMarkingShape,  // 30:横断歩道形状(縞模様毎)(子)
  FL8260_StopLine,               // 31:○停止線(親)
  FL8310_RoadEdge,               // 32:道路縁(親)
  FL8311_Curb,                   // 33:縁石(親)
  FA8312_Gutter,                 // 34:側溝(親)
  FA8313_GuardRail,              // 35:ガードレール(親)
  FA8314_ZebraZone,              // 36:ゼブラゾーン(親)
  FP8410_ShapeDescriptionPoint   // 37:形状記述点(子)
} DmpFtrClass_e;

// 関連(Relation)タイプ定義コード
typedef enum
{
  R_NONE = 0,  // 0: NONE
  R2110_Connectivity,
  R9110_Crossing,
  R9120_Adjacency,
  R9130_Branch,
  R9210_TrafficLightRegulationForLane,
  R9220_TrafficSignRegulationForLane,
  R9230_FeatureAlongWithLane
} DmpRelType_e;

// 属性(Attribute)種別定義コード
typedef enum
{
  A_NONE = 0,  // 0: NONE
  A_CT,  // 1:other textual content of traffic sign
  A_DA,  // 2:Diameter (数値, メートル)
  A_DH,  // 3:Divider Height (数値)
  A_DM,  // 4:Divider Marking (1:no line, 1:dashed line, ...)
  A_DT,  // 5:Divider Type (3:legal, 4:physical)
  A_DW,  // 6:Divider Width (数値)
  A_DX,  // 7:Divider Impact (1:双方向, 2:右から, 3:左から)
  A_DZ,  // 8:Divider Colour (1:white, 2:yellow)
  A_EV,  // 9:Lane Type (0:normal lane, 1:emergency lane, ...)
  A_GR,  // 10:Guard Rail type (0:板羽根, 1:パイプ)
  A_GU,  // 11:Gutter type (0:蓋なし, 1:フタあり, 2:グレーチング)
  A_HT,  // 12:Height (数値, メートル)
  A_LD,  // 13:Lane Dependent validity (L:左から車線をカウント, R:右から)
  A_LI,  // 14:Traffic Light Type (信号種別, 1:車両, 2:歩行者, 3:車両歩行者兼用, 9:その他)
  A_LM,  // 15:Measured Length (数値, メートル)
  A_LR,  // 16:Length of Road Element (数値)
  A_LY,  // 17:Lamp Type (ランプ種別, 1: 赤, 2:青, 3:黄, 4: 左矢印, 5: 直進矢印, 6:右矢印, 9:その他)
  A_NL,  // 18:Number of Lane (レーン数(交差点内は0))
  A_OY,  // 19:レーンの走行方向(0:順方向, 1:逆方向) 基本的には．
  A_PN,  // 20:pitch angle (数値 [deg])
  A_SP,  // 21:Speed (走行時の想定速度(本来の意味は速度制限))
  A_SY,  // 22:Symbol on Traffic Sign
  A_TR,  // 23:Toll Road (0:False, True)
  A_TS,  // 24:Traffic sign class (50:優先権, 51:方向, 55:停車禁止, 56:警告, ...)
  A_TT,  // 25:Travel time (数値)
  A_VT,  // 26:Vehicle Type (0:all, 11:Passenger cars, 15:emergency vehicle, 16:taxi, 24:bicycle, 25:pedestrian, ...)
  A_WI,  // 27:Width (数値)
  A_WL,  // 28:Width of Leftside (左幅員, 数値 [m])
  A_WR,  // 29:Width of Rightside (右幅員, 数値 [m])
  A_XM,  // 30:Pedestrian crossing marking type (0:外枠, 1:縞模様, 2:自転車通行帯)
  A_YN   // 31:yaw angle (数値 [deg])
} DmpAtrCode_e;

/****************************************
*  構造体定義
****************************************/

// 点(Point)データ定義
typedef struct
{
  double x;     // 平面直角座標系 Y座標(Ly) (東方向正) [m] (X,Yの入れ替わりに注意)
  double y;     // 平面直角座標系 X座標(Bx) (北方向正) [m] (X,Yの入れ替わりに注意)
  double z;     // 平面直角座標系 標高(H) [m]
  int    sno;   // 平面直角座標系 系番号 (1-19)
  double xLon;  // 経度 [deg]
  double yLat;  // 緯度 [deg]
} StDmpPoint_t;
typedef std::vector<StDmpPoint_t> VcDmpPoint_t;

//-----------------------------
// データベーステーブル読み込みバッファ定義
//-----------------------------

// attribute_composition
typedef struct
{
  int attribute_id;  // key
  int scope_level;
  int composition_number;
  int sequence_number;
  int parent_scope_level;
  int parent_comp_number;
  int parent_seq_number;
  std::string av_attr_value;
  DmpAtrCode_e av_type_code;
} StAttributeComposition_t;
// map<attribute_id, vector<struct>> 
typedef std::vector<StAttributeComposition_t> VcAttributeComposition_t;
typedef std::map<int, VcAttributeComposition_t> MpVcAttributeComposition_t;

// attribute_group (使用方法不明)
typedef struct
{
  int attribute_id;
} StAttributeGroup_t;
// vector<struct>
typedef std::vector<StAttributeGroup_t> VcAttributeGroup_t;

// cmp_feat_part_attr (未使用)
typedef struct
{
  int feat_category_num;  // index
  int feature_id;         // key
  int sequence_number;
  int attribute_id;
} StCmpFeatPartAttr_t;
// vector[feature_category_num-1].map<feature_id, vector<struct>>
typedef std::vector<std::map<int, std::vector<StCmpFeatPartAttr_t> > > VcMpVcCmpFeatPartAttr_t;

// feature_attr
typedef struct
{
  int feature_category_num;  // index
  int feature_id;            // key
  int attribute_id;
} StFeatureAttr_t;
// vector[feature_category_num-1].map<feature_id, struct>
typedef std::vector<std::map<int, StFeatureAttr_t> > VcMpFeatureAttr_t;

// feature_category (使わない)
typedef struct
{
  int feat_category_num;  // key
  std::string feat_category_name;
} StFeatureCategory_t;
// map<feat_category_num, struct>
typedef std::map<int, StFeatureCategory_t> MpFeatureCategory_t;

// feature_class_code (使えない)
typedef struct
{
  DmpFtrClass_e feature_class_code;  // key
  std::string description;
} StFeatureClassCode_t;
// map<feature_class_code, struct>
typedef std::map<int, StFeatureClassCode_t> MpFeatureClassCode_t;

// pf_area_feature
typedef struct
{
  int feat_category_num;  // =3
  int feature_id;  // key
  DmpFtrClass_e feature_class_code;
} StPfAreaFeature_t;
// map<feature_id, struct>
typedef std::map<int, StPfAreaFeature_t> MpPfAreaFeature_t;

// pf_area_topo_prim
typedef struct
{
  int feat_category_num;  // =3
  int feature_id;  // key
  int face_id;
} StPfAreaTopoPrim_t;
// map<feature_id, struct>
typedef std::map<int, StPfAreaTopoPrim_t> MpPfAreaTopoPrim_t;

// pf_comp_feat_part
typedef struct
{
  int comp_feat_category;  // =4
  int comp_feature_id;  // key
  int feature_number;
  int feature_category_num;
  int feature_id;
} StPfCompFeatPart_t;
// map<comp_feature_id, vector<struct>>
typedef std::map<int, std::vector<StPfCompFeatPart_t> > MpVcPfCompFeatPart_t;

// pf_comp_feature
typedef struct
{
  int feat_category_num;  // =4
  int feature_id;  // key
  DmpFtrClass_e feature_class_code;
  int from_feat_category;
  int from_feature_id;
  int to_feat_category;
  int to_feature_id;
} StPfCompFeature_t;
// map<feature_id, struct>
typedef std::map<int, StPfCompFeature_t> MpPfCompFeature_t;

// pf_line_feature
typedef struct
{
  int feat_category_num;  // =2
  int feature_id;  // key
  DmpFtrClass_e feature_class_code;
  int end_elevation;
  int from_feat_category;
  int from_feat_id;
  int to_feat_category;
  int to_feat_id;
} StPfLineFeature_t;
// map<feature_id, struct>
typedef std::map<int, StPfLineFeature_t> MpPfLineFeature_t;

// pf_line_topo_prim
typedef struct
{
  int feat_category_num;  // =2
  int feature_id;  // key
  int sequence_number;
  int edge_id;
  int edge_orientation;
  int start_elevation;
  int intermitted_elevation;
} StPfLineTopoPrim_t;
// map<feature_id, vector<struct>>
typedef std::map<int, std::vector<StPfLineTopoPrim_t> > MpVcPfLineTopoPrim_t;

// pf_point_feature
typedef struct
{
  int feat_category_num;  // =1
  int feature_id;  // key
  DmpFtrClass_e feature_class_code;
  int node_id;
} StPfPointFeature_t;
// map<feature_id, struct>
typedef std::map<int, StPfPointFeature_t> MpPfPointFeature_t;

// relation_feat_attr (未使用)
typedef struct
{
  int relationship_id;  // key
  int role_number;
  int feature_number;
  int attribute_id;
} StRelationFeatAttr_t;
// map<relationship_id, vector<struct>>
typedef std::map<int, std::vector<StRelationFeatAttr_t> > MpVcRelationFeatAttr_t;

// relation_role (使わない)
typedef struct
{
  DmpRelType_e rel_type_code;
  int role_number;
  std::string role_name;
  bool repeatable;
  bool mandatory;
} StRelationRole_t;
// map<rel_type_code, vector<struct>>
typedef std::map<int, std::vector<StRelationRole_t> > MpVcRelationRole_t;

// relation_type_code (使えない)
typedef struct
{
  DmpRelType_e rel_type_code;
  std::string description;
} StRelationTypeCode_t;
// map<rel_type_code, struct>
typedef std::map<int, StRelationTypeCode_t> MpRelationTypeCode_t;

// relationship
typedef struct
{
  int relationship_id;
  DmpRelType_e rel_type;
} StRelationship_t;
// map<relationship_id, struct>
typedef std::map<int, StRelationship_t> MpRelationship_t;

// relationship_attr
typedef struct
{
  int relationship_id;
  int attribute_id;
} StRelationshipAttr_t;
// map<relationship_id, vector<struct>>
typedef std::map<int, std::vector<StRelationshipAttr_t> > MpVcRelationshipAttr_t;

// relationship_feat
typedef struct
{
  int relationship_id;
  int role_number;
  int feature_number;
  int feat_category_num;
  int feature_id;
  DmpRelType_e rel_type;  // relationshipからコピーする
} StRelationshipFeat_t;
// map<relationship_id, vector<struct>>
typedef std::vector<StRelationshipFeat_t> VcRelationshipFeat_t;
typedef std::map<int, VcRelationshipFeat_t> MpVcRelationshipFeat_t;

// st_edge
typedef struct
{
  int edge_id;
  int start_node;
  int end_node;
  int next_left_edge;
  int next_right_edge;
  int left_face;
  int right_face;
  VcDmpPoint_t geometry;  // geometry(LineStringZ,2449)
} StStEdge_t;
// map<edge_id, struct>
typedef std::map<int, StStEdge_t> MpStEdge_t;

// st_face
typedef struct
{
  int face_id;
  VcDmpPoint_t mbr;  // geometry(PolygonZ,2449),
  double slope;
  int aspect;
  int light_value;
  bool extended_geom_flag;
} StStFace_t;
// map<face_id, struct>
typedef std::map<int, StStFace_t> MpStFace_t;

// st_node
typedef struct
{
  int node_id;
  StDmpPoint_t geometry;  // geometry(PointZ,2449),
  int containing_face;
} StStNode_t;
// map<node_id, struct>
typedef std::map<int, StStNode_t> MpStNode_t;

//-----------------------------
// 地物データ再構築バッファ定義
//-----------------------------

//●[FP4120]ジャンクション(Junction)定義
typedef struct
{
 // pf_point_feature
  //int feat_category_num;  // 地物カテゴリ =1:Point
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='4120'
  int node_id;              // st_nodeへの参照キー

 // st_node
  StDmpPoint_t sn_geometry;  // 座標情報

 // attribute

} StDmpJunction_t;
// map<feature_id, struct>
typedef std::map<int, StDmpJunction_t> MpDmpJunction_t;

//●[FC8110]レーン(Lane)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8110'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =2:Line
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 // pf_comp_feat_part - LaneLine
  //int pcfp1_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp1_comp_feature_id;     // = 本体(親) 地物ID
  //int pcfp1_feature_number;      // 何番目の部品かを表す
  //int pcfp1_feature_category_num;  // 部品(子)地物カテゴリ =2:Line
  int pcfp1_feature_id;            // 部品(子)地物ID

 // pf_line_feature - LaneLine
  //int plf1_feat_category_num;   // 部品(子)地物カテゴリ =2:Line
  //int plf1_feature_id;          // 部品(子)地物ID
  //DmpFtrClass_e plf1_feature_class_code;  // 地物クラス ='8130'
  //int plf1_end_elevation;       // 標高 
  //int plf1_from_feat_category;  // 始点の地物カテゴリ =1:Point
  //int plf1_from_feat_id;        // 始点の地物ID
  //int plf1_to_feat_category;    // 終点の地物カテゴリ =1:Point
  //int plf1_to_feat_id;          // 終点の地物ID

 // pf_line_topo_prim - LaneLine
  //int pltp1_feat_category_num;  // 部品(子)地物カテゴリ =2:Line
  //int pltp1_feature_id;         // 部品(子)地物ID
  //int pltp1_sequence_number;    // 複数edgeがある場合用 =1
  int pltp1_edge_id;              // st_edgeへの参照キー
  //int pltp1_edge_orientation;   // 0:CW,1:CCW =0
  //int pltp1_start_elevation;    // 標高
  //int pltp1_intermitted_elevation;  // 標高

 // st_edge - LaneLine
  //int se1_edge_id;          // ID
  //int se1_start_node;       // 始点node_id
  //int se1_end_node;         // 終点node_id
  //int se1_next_left_edge;   // 隣接する左エッジID
  //int se1_next_right_edge;  // 隣接する右エッジID
  //int se1_left_face;        // 左面ID
  //int se1_right_face;       // 右面ID
  VcDmpPoint_t se1_geometry;  // 点の系列

 // pf_comp_feat_part - LaneArea
  //int pcfp2_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp2_comp_feature_id;     // = 本体(親) 地物ID
  //int pcfp2_feature_number;      // 何番目の部品かを表す
  //int pcfp2_feature_category_num;  // 部品(子)地物カテゴリ =3:Area
  int pcfp2_feature_id;            // 部品(子)地物ID 

 // pf_area_feature - LaneArea
  //int paf2_feat_category_num;   // 部品(子)地物カテゴリ =3:Area
  //int paf2_feature_id;          // 部品(子)地物ID
  //DmpFtrClass_e paf2_feature_class_code;  // 地物クラス ='8120'

 // pf_area_topo_prim - LaneArea
  //int patp2_feat_category_num;  // 部品(子)地物カテゴリ =3:Area
  //int patp2_feature_id;         // 部品(子)地物ID
  int patp2_face_id;              // st_faceへの参照キー

 // st_face - LaneArea
  VcDmpPoint_t sf2_mbr;            // 領域を表す多角形

 // attribute
  int    iA_EV;  // Lane Type
  double dA_LR;  // Length of Road Element
  int    iA_NL;  // Number of Lane
  int    iA_OY;  // レーンの走行方向(0:順方向, 1:逆方向)
  double dA_PN;  // pitch angle
  int    iA_SP;  // Speed
  int    iA_TR;  // Toll Road
  int    iA_VT;  // Vehicle Type
  double dA_WI;  // Width
  double dA_WL;  // Width of Leftside
  double dA_WR;  // Width of Rightside
  double dA_YN;  // yaw angle
} StDmpLane_t;
// map<feature_id, struct>
typedef std::map<int, StDmpLane_t> MpDmpLane_t;

//●[FA8150]歩道領域(PedestrianArea)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8150'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;    // 地物ID
  int patp_face_id;         // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;      // 領域を表す多角形

 // attribute

} StDmpPedestrianArea_t;
typedef std::vector<StDmpPedestrianArea_t> VcDmpPedestrianArea_t;

//●[FL8160]歩道中心線(PedestrianLine)定義
typedef struct
{
 // pf_line_feature
  //int feat_category_num;  // 地物カテゴリ =2:Line
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8160'
  //int end_elevation;
  //int from_feat_category;
  //int from_feat_id;
  //int to_feat_category;
  //int to_feat_id;

 // pf_line_topo_prim
  //int feat_category_num;  // 地物カテゴリ =2:Line
  //int feature_id;         // 地物ID (キー)
  //int pltp_sequence_number;   // 複数edgeがある場合用 =1
  int pltp_edge_id;         // st_edgeへの参照キー
  //int pltp_edge_orientation;  // 0:CW,1:CCW =0
  //int pltp_start_elevation;   // 標高
  //int pltp_intermitted_elevation;  // 標高

 // st_edge
  //int se_edge_id;          // ID
  //int se_start_node;       // 始点node_id
  //int se_end_node;         // 終点node_id
  //int se_next_left_edge;   // 隣接する左エッジID
  //int se_next_right_edge;  // 隣接する右エッジID
  //int se_left_face;        // 左面ID
  //int se_right_face;       // 右面ID
  VcDmpPoint_t se_geometry;  // 点の系列

 // attribute

} StDmpPedestrianLine_t;
typedef std::vector<StDmpPedestrianLine_t> VcDmpPedestrianLine_t;

//●[FC8140]歩道(ExtendedPedestrian)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8140'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =2:Line
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 // pf_comp_feat_part以下 - PedestrianLine
  VcDmpPedestrianLine_t mVcDmpPedestrianLine;  // [FL8160]歩道中心線

 // pf_comp_feat_part以下 - PedestrianArea
  VcDmpPedestrianArea_t mVcDmpPedestrianArea;  // [FA8150]歩道領域

 // attribute

} StDmpExtendedPedestrian_t;
// map<feature_id, struct>
typedef std::map<int, StDmpExtendedPedestrian_t> MpDmpExtendedPedestrian_t;

//●[FA8252]横断歩道模様(PedestrianCrossingMarkingsShape)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8252'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;  // 地物ID
  int patp_face_id;       // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;  // 領域を表す多角形

 // attribute
  int iA_XM;  // Pedestrian crossing marking type 
} StDmpPedestrianCrossingMarkingsShape_t;
typedef std::vector<StDmpPedestrianCrossingMarkingsShape_t> VcDmpPedestrianCrossingMarkingsShape_t;

//●[FC8145]横断歩道(ExtendedPedestrianCrossing)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8145'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =2:Line
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 // pf_comp_feat_part以下 - PedestrianLine
  VcDmpPedestrianLine_t mVcDmpPedestrianLine;  // [FL8160]歩道中心線

 // pf_comp_feat_part以下 - PedestrianArea
  VcDmpPedestrianArea_t mVcDmpPedestrianArea;  // [FA8150]歩道領域

 // pf_comp_feat_part以下 - PedestrianCrossingMarkingsShape
  VcDmpPedestrianCrossingMarkingsShape_t mVcDmpPedestrianCrossingMarkingsShape;  // [FA8252]横断歩道模様

 // attribute

} StDmpExtendedPedestrianCrossing_t;
// map<feature_id, struct>
typedef std::map<int, StDmpExtendedPedestrianCrossing_t> MpDmpExtendedPedestrianCrossing_t;

//●[FA8170]交差点領域(IntersectionAreaShape)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8170'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;    // 地物ID
  int patp_face_id;         // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;      // 領域を表す多角形

 // attribute

} StDmpIntersectionAreaShape_t;
// map<feature_id, struct>
typedef std::map<int, StDmpIntersectionAreaShape_t> MpDmpIntersectionAreaShape_t;

//●[FP8210]ポール(Pole)定義
typedef struct
{
 // pf_point_feature
  //int feat_category_num;  // 地物カテゴリ =1:Point
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8210'
  int node_id;              // st_nodeへの参照キー

 // st_node
  StDmpPoint_t sn_geometry;  // 座標情報

 // attribute
  double dA_DA;  // Diameter
  double dA_LM;  // Measured Length
  double dA_PN;  // pitch angle
  double dA_YN;  // yaw angle
} StDmpPole_t;
typedef std::vector<StDmpPole_t> VcDmpPole_t;
// map<feature_id, struct>
typedef std::map<int, StDmpPole_t> MpDmpPole_t;

//●[FC8220]標識(ExtendedTrafficSign)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8230'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =2:Line
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 // pf_comp_feat_part以下 - Pole
  VcDmpPole_t mVcDmpPole;    // [FP8210]ポール

 // pf_comp_feat_part - Traffic Sign
  //int pcfp2_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp2_comp_feature_id;   // = 本体(親) 地物ID
  //int pcfp2_feature_number;    // 何番目の部品かを表す
  //int pcfp2_feature_category_num;  // 部品(子)地物カテゴリ =1:Point
  int pcfp2_feature_id;          // 部品(子)地物ID

 // pf_point_feature - Traffic Sign
  //int ppf2_feat_category_num;  // 地物カテゴリ =1:Point
  //int ppf2_feature_id;         // 地物ID (キー)
  //DmpFtrClass_e ppf2_feature_class_code;  // 地物クラス ='7220'
  int ppf2_node_id;              // st_nodeへの参照キー

 // st_node - Traffic Sign
  StDmpPoint_t sn2_geometry;     // 座標情報

 // attribute
  int    iA_CT;   // other textual content of traffic sign
  double dA_PN;   // pitch angle
  int    iA_SY;   // Symbol on Traffic Sign
  int    iA_TS;   // Traffic sign class
  double dA_YN;   // yaw angle
} StDmpExtendedTrafficSign_t;
// map<feature_id, struct>
typedef std::map<int, StDmpExtendedTrafficSign_t> MpDmpExtendedTrafficSign_t;

//●[FP8231]信号ランプ(TrafficLightLamp)定義
typedef struct
{
 // pf_point_feature
  //int feat_category_num;  // 地物カテゴリ =1:Point
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8231'
  int node_id;              // st_nodeへの参照キー

 // st_node
  StDmpPoint_t sn_geometry;  // 座標情報

 // Attrubute
  int iA_LY;  // ランプ種別
} StDmpTrafficLightLamp_t;
typedef std::vector<StDmpTrafficLightLamp_t> VcDmpTrafficLightLamp_t;

//●[FC8230]信号(ExtendedTrafficLight)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8230'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =2:Line
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 //○pf_comp_feat_part以下 - Pole
  VcDmpPole_t mVcDmpPole;    // [FP8210]ポール

 //○pf_comp_feat_part以下 - TrafficLightLamp
  VcDmpTrafficLightLamp_t mVcTrafficLightLamp;  // [FP8231]信号ランプ

 // attribute
  int    iA_LI;   // Traffic Light Type
  double dA_PN;   // pitch angle
  double dA_YN;   // yaw angle
} StDmpExtendedTrafficLight_t;
// map<feature_id, struct>
typedef std::map<int, StDmpExtendedTrafficLight_t> MpDmpExtendedTrafficLight_t;

//●[FC8240]街灯(ExtendedLighting)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8240'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =2:Line
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 //○pf_comp_feat_part - Lighting
  //int pcfp1_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp1_comp_feature_id;   // = 本体(親) 地物ID
  //int pcfp1_feature_number;    // 何番目の部品かを表す
  //int pcfp1_feature_category_num;  // 部品(子)地物カテゴリ =1:Point
  int pcfp1_feature_id;          // 部品(子)地物ID

 // pf_point_feature - Lighting
  //int ppf1_feat_category_num;  // 地物カテゴリ =1:Point
  //int ppf1_feature_id;         // 地物ID (キー)
  //DmpFtrClass_e ppf1_feature_class_code;  // 地物クラス ='7252'
  int ppf1_node_id;              // st_nodeへの参照キー

 // st_node - Lighting
  StDmpPoint_t sn1_geometry;     // 座標情報

 //○pf_comp_feat_part - Lighting Lamp
  //int pcfp2_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp2_comp_feature_id;   // = 本体(親) 地物ID
  //int pcfp2_feature_number;    // 何番目の部品かを表す
  //int pcfp2_feature_category_num;  // 部品(子)地物カテゴリ =2:Line
  int pcfp2_feature_id;          // 部品(子)地物ID

 // pf_line_feature - Lighting Lamp
  //int plf2_feat_category_num;  // 地物カテゴリ =2:Line
  //int plf2_feature_id;         // 地物ID (キー)
  //DmpFtrClass_e plf2_feature_class_code;  // 地物クラス ='8241'
  //int plf2_end_elevation;
  //int plf2_from_feat_category;
  //int plf2_from_feat_id;
  //int plf2_to_feat_category;
  //int plf2_to_feat_id;

 // pf_line_topo_prim - Lighting Lamp
  //int pltp2_feat_category_num;  // 地物カテゴリ =2:Line
  //int pltp2_feature_id;         // 地物ID (キー)
  //int pltp2_sequence_number;    // 複数edgeがある場合用 =1
  int pltp2_edge_id;              // st_edgeへの参照キー
  //int pltp2_edge_orientation;   // 0:CW,1:CCW =0
  //int pltp2_start_elevation;    // 標高
  //int pltp2_intermitted_elevation;  // 標高

 // st_edge - Lighting Lamp
  //int se2_edge_id;          // ID
  //int se2_start_node;       // 始点node_id
  //int se2_end_node;         // 終点node_id
  //int se2_next_left_edge;   // 隣接する左エッジID
  //int se2_next_right_edge;  // 隣接する右エッジID
  //int se2_left_face;        // 左面ID
  //int se2_right_face;       // 右面ID
  VcDmpPoint_t se2_geometry;  // 点の系列

 //○pf_comp_feat_part - Pole
  //int pcfp3_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp3_comp_feature_id;   // = 本体(親) 地物ID
  //int pcfp3_feature_number;    // 何番目の部品かを表す
  //int pcfp3_feature_category_num;  // 部品(子)地物カテゴリ =1:Point
  int pcfp3_feature_id;          // 部品(子)地物ID

 // pf_point_feature - Pole
  //int ppf3_feat_category_num;  // 地物カテゴリ =1:Point
  //int ppf3_feature_id;         // 地物ID (キー)
  //DmpFtrClass_e ppf3_feature_class_code;  // 地物クラス ='8210'
  int ppf3_node_id;              // st_nodeへの参照キー

 // st_node - Pole
  StDmpPoint_t sn3_geometry;  // 座標情報

 // attribute - Pole
  double dA3_DA;  // Diameter
  double dA3_LM;  // Measured Length
  double dA3_PN;  // pitch angle
  double dA3_YN;  // yaw angle

 // attribute

} StDmpExtendedLighting_t;
// map<feature_id, struct>
typedef std::map<int, StDmpExtendedLighting_t> MpDmpExtendedLighting_t;

//●[FA8251]路面マーク模様(RoadMarkingsShape)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8251'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;    // 地物ID
  int patp_face_id;         // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;      // 領域を表す多角形

 // attribute

} StDmpRoadMarkingsShape_t;
typedef std::vector<StDmpRoadMarkingsShape_t> VcDmpRoadMarkingsShape_t;

//●[FC8250]路面マーク(ExtendedRoadMarkings)定義
typedef struct
{
 // pf_comp_feature
  //int feat_category_num;   // 地物カテゴリ =4:Complex
  int feature_id;            // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8250'
  //int from_feat_category;  // この地物を構成する最初の地物カテゴリ =1:Point
  //int from_feature_id;     // この地物を構成する最初の地物ID
  //int to_feat_category;    // この地物を構成する最後の地物カテゴリ =3:Area
  //int to_feature_id;       // この地物を構成する最後の地物ID

 //○pf_comp_feat_part - RoadMarkings
  //int pcfp1_comp_feat_category;  // = 本体(親)地物カテゴリ =4:Complex
  //int pcfp1_comp_feature_id;   // = 本体(親) 地物ID
  //int pcfp1_feature_number;    // 何番目の部品かを表す
  //int pcfp1_feature_category_num;  // 部品(子)地物カテゴリ =1:Point
  int pcfp1_feature_id;          // 部品(子)地物ID

 // pf_point_feature - RoadMarkings
  //int ppf1_feat_category_num;  // 地物カテゴリ =1:Point
  //int ppf1_feature_id;         // 地物ID (キー)
  //DmpFtrClass_e ppf1_feature_class_code;  // 地物クラス ='7254'
  int ppf1_node_id;              // st_nodeへの参照キー

 // st_node - RoadMarkings
  StDmpPoint_t sn1_geometry;     // 座標情報

 //○pf_comp_feat_part以下 - RoadMarkingsShape
  VcDmpRoadMarkingsShape_t mVcDmpRoadMarkingsShape;  // [FA8251]路面マーク模様

 // attribute

} StDmpExtendedRoadMarkings_t;
// map<feature_id, struct>
typedef std::map<int, StDmpExtendedRoadMarkings_t> MpDmpExtendedRoadMarkings_t;

//●[FL8260]停止線(StopLine)定義
typedef struct
{
 // pf_line_feature
  //int feat_category_num;  // 地物カテゴリ =2:Line
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8260'
  //int end_elevation;
  //int from_feat_category;
  //int from_feat_id;
  //int to_feat_category;
  //int to_feat_id;

 // pf_line_topo_prim
  //int feat_category_num;  // 地物カテゴリ =2:Line
  //int feature_id;         // 地物ID (キー)
  //int pltp_sequence_number;   // 複数edgeがある場合用 =1
  int pltp_edge_id;             // st_edgeへの参照キー
  //int pltp_edge_orientation;  // 0:CW,1:CCW =0
  //int pltp_start_elevation;   // 標高
  //int pltp_intermitted_elevation;  // 標高

 // st_edge
  //int se_edge_id;          // ID
  //int se_start_node;       // 始点node_id
  //int se_end_node;         // 終点node_id
  //int se_next_left_edge;   // 隣接する左エッジID
  //int se_next_right_edge;  // 隣接する右エッジID
  //int se_left_face;        // 左面ID
  //int se_right_face;       // 右面ID
  VcDmpPoint_t se_geometry;  // 点の系列

 // attribute

} StDmpStopLine_t;
// map<feature_id, struct>
typedef std::map<int, StDmpStopLine_t> MpDmpStopLine_t;

//●[FL8310]道路縁(RoadEdge)定義
typedef struct
{
 // pf_line_feature
  //int feat_category_num;  // 地物カテゴリ =2:Line
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8310'
  //int end_elevation;
  //int from_feat_category;
  //int from_feat_id;
  //int to_feat_category;
  //int to_feat_id;

 // pf_line_topo_prim
  //int feat_category_num;  // 地物カテゴリ =2:Line
  //int feature_id;         // 地物ID (キー)
  //int pltp_sequence_number;   // 複数edgeがある場合用 =1
  int pltp_edge_id;             // st_edgeへの参照キー
  //int pltp_edge_orientation;  // 0:CW,1:CCW =0
  //int pltp_start_elevation;   // 標高
  //int pltp_intermitted_elevation;  // 標高

 // st_edge
  //int se_edge_id;          // ID
  //int se_start_node;       // 始点node_id
  //int se_end_node;         // 終点node_id
  //int se_next_left_edge;   // 隣接する左エッジID
  //int se_next_right_edge;  // 隣接する右エッジID
  //int se_left_face;        // 左面ID
  //int se_right_face;       // 右面ID
  VcDmpPoint_t se_geometry;  // 点の系列

 // attribute

} StDmpRoadEdge_t;
// map<feature_id, struct>
typedef std::map<int, StDmpRoadEdge_t> MpDmpRoadEdge_t;

//●[FL8311]縁石(Curb)定義
typedef struct
{
 // pf_line_feature
  //int feat_category_num;  // 地物カテゴリ =2:Line
  int feature_id;           // 地物ID (キー)
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8311'
  //int end_elevation;
  //int from_feat_category;
  //int from_feat_id;
  //int to_feat_category;
  //int to_feat_id;

 // pf_line_topo_prim
  //int feat_category_num;  // 地物カテゴリ =2:Line
  //int feature_id;         // 地物ID (キー)
  //int pltp_sequence_number;   // 複数edgeがある場合用 =1
  int pltp_edge_id;             // st_edgeへの参照キー
  //int pltp_edge_orientation;  // 0:CW,1:CCW =0
  //int pltp_start_elevation;   // 標高
  //int pltp_intermitted_elevation;  // 標高

 // st_edge
  //int se_edge_id;          // ID
  //int se_start_node;       // 始点node_id
  //int se_end_node;         // 終点node_id
  //int se_next_left_edge;   // 隣接する左エッジID
  //int se_next_right_edge;  // 隣接する右エッジID
  //int se_left_face;        // 左面ID
  //int se_right_face;       // 右面ID
  VcDmpPoint_t se_geometry;  // 点の系列

 // attribute
  double dA_HT;  // Height
  double dA_WI;  // Width
} StDmpCurb_t;
// map<feature_id, struct>
typedef std::map<int, StDmpCurb_t> MpDmpCurb_t;

//●[FA8312]側溝(Gutter)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8312'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;  // 地物ID
  int patp_face_id;       // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;  // 領域を表す多角形

 // attribute
  int iA_GU;  // Gutter type
} StDmpGutter_t;
// map<feature_id, struct>
typedef std::map<int, StDmpGutter_t> MpDmpGutter_t;

//●[FA8313]ガードレール(GuardRail)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8313'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;  // 地物ID
  int patp_face_id;       // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;  // 領域を表す多角形

 // attribute
  int iA_GR;  // Guard Rail type
} StDmpGuardRail_t;
// map<feature_id, struct>
typedef std::map<int, StDmpGuardRail_t> MpDmpGuardRail_t;

//●[FA8314]ゼブラゾーン(ZebraZone)定義
typedef struct
{
 // pf_area_feature
  //int feat_category_num;  // 地物カテゴリ =3:Area
  int feature_id;           // 地物ID
  //DmpFtrClass_e feature_class_code;  // 地物クラス ='8314'

 // pf_area_topo_prim
  //int patp_feat_category_num;  // 地物カテゴリ =3:Area
  //int patp_feature_id;  // 地物ID
  int patp_face_id;       // st_faceへの参照キー

 // st_face
  VcDmpPoint_t sf_mbr;  // 領域を表す多角形

 // attribute

} StDmpZebraZone_t;
// map<feature_id, struct>
typedef std::map<int, StDmpZebraZone_t> MpDmpZebraZone_t;

//-----------------------------
// 関連データ再構築バッファ定義
//-----------------------------
typedef struct
{
  int role_number;
  int feat_category_num;
  int feature_id;
} StDmpRebRelationshipFeat_t;
typedef std::vector<StDmpRebRelationshipFeat_t> VcDmpRebRelationshipFeat_t;

typedef struct
{
  int relationship_id;
  DmpRelType_e rel_type;
  VcDmpRebRelationshipFeat_t mVcDmpRebRelationshipFeat;
} StDmpRebRelationship_t;
typedef std::vector<StDmpRebRelationship_t> VcDmpRebRelationship_t;

#endif // _DMP_DATA_H_
