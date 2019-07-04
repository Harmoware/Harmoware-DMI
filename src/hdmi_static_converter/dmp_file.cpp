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
/****************************************
*  dmp_file.cpp
*  データベース形式ダイナミックマップの読み込み処理
****************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <math.h>

#include "dmp_data.h"
#include "dmp_file.h"
#include "commonlib.h"

#ifdef USE_DMLIB
 #include <is/DmManager.h>
#else
 #include <libpq-fe.h>
#endif

/************************************************************
*  グローバル変数定義
************************************************************/

extern bool bDebugFlg;        // デバッグログ出力フラグ
extern std::ofstream ofsdbg;  // デバッグログ出力ストリーム

// 地物データ読み込みバッファ
MpDmpJunction_t g_MpDmpJunction;  // [FP4120]ジャンクション
MpDmpLane_t g_MpDmpLane;          // [FC8110]レーン
MpDmpExtendedPedestrian_t g_MpDmpExtendedPedestrian;  // [FC8140]歩道
MpDmpExtendedPedestrianCrossing_t g_MpDmpExtendedPedestrianCrossing;  // [FC8145]横断歩道
MpDmpIntersectionAreaShape_t g_MpDmpIntersectionAreaShape;  // [FA8170]交差点領域
MpDmpExtendedTrafficLight_t g_MpDmpExtendedTrafficLight;  // [FC8230]信号
MpDmpPole_t g_MpDmpPole;                                  // [FP8210]ポール
MpDmpExtendedTrafficSign_t g_MpDmpExtendedTrafficSign;    // [FC8220]標識
MpDmpExtendedLighting_t g_MpDmpExtendedLighting;          // [FC8240]街灯
MpDmpExtendedRoadMarkings_t g_MpDmpExtendedRoadMarkings;  // [FC8250]道路標示
MpDmpStopLine_t g_MpDmpStopLine;    // [FL8260]停止線
MpDmpRoadEdge_t g_MpDmpRoadEdge;    // [FL8310]道路縁
MpDmpCurb_t g_MpDmpCurb;            // [FL8311]縁石
MpDmpGutter_t g_MpDmpGutter;        // [FA8312]側溝
MpDmpGuardRail_t g_MpDmpGuardRail;  // [FA8313]ガードレール
MpDmpZebraZone_t g_MpDmpZebraZone;  // [FA8314]ゼブラゾーン

// 関連データ再構築バッファ
VcDmpRebRelationship_t g_VcDmpRebRelationshipConnectivity;  // [R2110]経路接続
VcDmpRebRelationship_t g_VcDmpRebRelationshipCrossing;      // [R9110]交差
VcDmpRebRelationship_t g_VcDmpRebRelationshipAdjacency;     // [R9120]隣接
VcDmpRebRelationship_t g_VcDmpRebRelationshipBranch;        // [R9130]分岐
VcDmpRebRelationship_t g_VcDmpRebRelationshipTrafficLightRegulationForLane;  // [R9210]信号規制
VcDmpRebRelationship_t g_VcDmpRebRelationshipTrafficSignRegulationForLane;  // [R9220]標識規制
VcDmpRebRelationship_t g_VcDmpRebRelationshipFeatureAlongWithLane;  // [R9230]地物関連

/************************************************************
*  ローカル変数定義
************************************************************/
namespace
{
  // データベーステーブル読み込みバッファ
  MpVcAttributeComposition_t mMpVcAttributeComposition;  // attribute_composition
  VcAttributeGroup_t mVcAttributeGroup;      // attribute_group
  VcMpVcCmpFeatPartAttr_t mVcMpVcCmpFeatPartAttr(4);  // cmp_feat_part_attr
  VcMpFeatureAttr_t mVcMpFeatureAttr(4);     // feature_attr
  MpFeatureCategory_t mMpFeatureCategory;    // feature_category
  MpFeatureClassCode_t mMpFeatureClassCode;  // feature_class_code
  MpPfAreaFeature_t mMpPfAreaFeature;        // pf_area_feature
  MpPfAreaTopoPrim_t mMpPfAreaTopoPrim;      // pf_area_topo_prim
  MpVcPfCompFeatPart_t mMpVcPfCompFeatPart;  // pf_comp_feat_part
  MpPfCompFeature_t mMpPfCompFeature;        // pf_comp_feature
  MpPfLineFeature_t mMpPfLineFeature;        // pf_line_feature
  MpVcPfLineTopoPrim_t mMpVcPfLineTopoPrim;  // pf_line_topo_prim
  MpPfPointFeature_t mMpPfPointFeature;      // pf_point_feature
  MpVcRelationFeatAttr_t mMpVcRelationFeatAttr;  // relation_feat_attr
  MpVcRelationRole_t mMpVcRelationRole;      // relation_role
  MpRelationTypeCode_t mMpRelationTypeCode;  // relation_type_code
  MpRelationship_t mMpRelationship;          // relationship
  MpVcRelationshipAttr_t mMpVcRelationshipAttr;  // relationship_attr
  MpVcRelationshipFeat_t mMpVcRelationshipFeat;  // relationship_feat
  MpStEdge_t mMpStEdge;  // st_edge
  MpStFace_t mMpStFace;  // st_face
  MpStNode_t mMpStNode;  // st_node

  // 複合地物として処理済の地物ID
  std::set<int> mCmpDonePoint;  // 処理済みPoint型feature_id
  std::set<int> mCmpDoneLine;   // 処理済みLine型feature_id
  std::set<int> mCmpDoneArea;   // 処理済みArea型feature_id
}

/************************************************************
*  ローカル関数定義
************************************************************/

static DmpFtrClass_e getFeatureClassCode(const char *ch);
static DmpRelType_e getRelationshipType(const char *ch);
static DmpAtrCode_e getAttributeCode(const char *ch);

static bool getGeometryPointZ(std::string str, int sno, StDmpPoint_t& pbuff);
static bool getGeometryLinestringZ(std::string str, int sno, VcDmpPoint_t& lbuff);
static bool getGeometryPolygonZ(std::string str, int sno, VcDmpPoint_t& abuff);
static VcAttributeComposition_t *getFeatureAttribute(DmpFtrCat_e fcat, int fid);
static void calXyToBl(double x, double y, int sno, double& phi, double& lmd);
//static void calBlToXy(double phi, double lmd, int sno, double& x, double& y);

/************************************************************
*  データベース形式 静的ダイナミックマップの読み込み
*
*    input: host  - PostgreSQLサーバ ホスト名
*           port  - PostgreSQLサーバ ポート番号
*           user  - PostgreSQLサーバ ユーザ名 
*           pass  - PostgreSQLサーバ パスワード 
*           dbase - PostgreSQLデータベース名
*           sno   - 平面直角座標系の系番号(1-19)
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
bool readDmpData(std::string host, int port, std::string user, std::string pass, std::string dbname, int sno)
{
#ifdef USE_DMLIB
  ResultSet rs;  // クエリ結果
#else
  PGresult *res;  // クエリ結果
  int nFields;    // 取得したデータの桁数
  int nTuples;    // 取得したデータの行数
#endif

  //std::cerr << "DEBUG:[readDmpData] START" << std::endl;

  // 作業領域の初期化
  mMpVcAttributeComposition.clear();
  mVcAttributeGroup.clear();
  mVcMpVcCmpFeatPartAttr[0].clear();
  mVcMpVcCmpFeatPartAttr[1].clear();
  mVcMpVcCmpFeatPartAttr[2].clear();
  mVcMpVcCmpFeatPartAttr[3].clear();
  mVcMpFeatureAttr[0].clear();
  mVcMpFeatureAttr[1].clear();
  mVcMpFeatureAttr[2].clear();
  mVcMpFeatureAttr[3].clear();
  mMpFeatureCategory.clear();
  mMpFeatureClassCode.clear();
  mMpPfAreaFeature.clear();
  mMpPfAreaTopoPrim.clear();
  mMpVcPfCompFeatPart.clear();
  mMpPfCompFeature.clear();
  mMpPfLineFeature.clear();
  mMpVcPfLineTopoPrim.clear();
  mMpPfPointFeature.clear();
  mMpVcRelationFeatAttr.clear();
  mMpVcRelationRole.clear();
  mMpRelationTypeCode.clear();
  mMpRelationship.clear();
  mMpVcRelationshipAttr.clear();
  mMpVcRelationshipFeat.clear();
  mMpStEdge.clear();
  mMpStFace.clear();
  mMpStNode.clear();

  g_MpDmpJunction.clear();
  g_MpDmpLane.clear();
  g_MpDmpExtendedPedestrian.clear();
  g_MpDmpExtendedPedestrianCrossing.clear();
  g_MpDmpIntersectionAreaShape.clear();
  g_MpDmpExtendedTrafficLight.clear();
  g_MpDmpPole.clear();
  g_MpDmpExtendedTrafficSign.clear();
  g_MpDmpExtendedLighting.clear();
  g_MpDmpExtendedRoadMarkings.clear();
  g_MpDmpStopLine.clear();
  g_MpDmpRoadEdge.clear();
  g_MpDmpCurb.clear();
  g_MpDmpGutter.clear();
  g_MpDmpGuardRail.clear();
  g_MpDmpZebraZone.clear();

  g_VcDmpRebRelationshipConnectivity.clear();
  g_VcDmpRebRelationshipCrossing.clear();
  g_VcDmpRebRelationshipAdjacency.clear();
  g_VcDmpRebRelationshipBranch.clear();
  g_VcDmpRebRelationshipTrafficLightRegulationForLane.clear();
  g_VcDmpRebRelationshipTrafficSignRegulationForLane.clear();
  g_VcDmpRebRelationshipFeatureAlongWithLane.clear();

  mCmpDonePoint.clear();
  mCmpDoneLine.clear();
  mCmpDoneArea.clear();

#ifdef USE_DMLIB
  // DBシステムに接続
  Connection* con;
  try
  {
    con = DmManager::getDBConnection(host.c_str(), port);
    std::cerr << "DEBUG:[readDmpData] connect DBsystem" << std::endl;
  }
  catch(ConnectionFailedException ex)
  {
    std::cerr << "ERROR:[readDmpData-01] getDBConnection(), ConnectionFailedException" << std::endl;
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-02] getDBConnection(), Exception" << std::endl;
    return false;
  }
#else
  // データベース接続パラメータ作成
  char conninfo[256];
  snprintf(conninfo, 256, "host=%s port=%d user=%s password=%s dbname=%s",
           host.c_str(), port, user.c_str(), pass.c_str(), dbname.c_str());
  std::cerr << "DEBUG:[readDmpData] conninfo=[" << std::string(conninfo) << "]" << std::endl;

  // データベース接続
  PGconn *conn = PQconnectdb(conninfo);
  if(PQstatus(conn) != CONNECTION_OK)
  {
    std::cerr << "ERROR:[readDmpData-02] Connection failed, err=" << std::string(PQerrorMessage(conn)) << std::endl;
    return false;
  }
#endif

  //---------------------------
  // データの読み込みと解析
  //---------------------------

  // attribute_composition
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.attribute_composition");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-attribute_composition-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-attribute_composition-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-attribute_composition-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StAttributeComposition_t stBuff;
    stBuff.attribute_id = rs.getInt(0);
    stBuff.scope_level = rs.getInt(1);
    stBuff.composition_number = rs.getInt(2);
    stBuff.sequence_number = rs.getInt(3);
    stBuff.parent_scope_level = rs.getInt(4);
    stBuff.parent_comp_number = rs.getInt(5);
    stBuff.parent_seq_number = rs.getInt(6);
    stBuff.av_attr_value = rs.getString(7);
    stBuff.av_type_code = getAttributeCode(rs.getString(8).c_str());
#else
  res = PQexec(conn, "SELECT * FROM coi.attribute_composition");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-attribute_composition-04] PQexec(attribute_composition), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 9)
  {
    std::cerr << "ERROR:[readDmpData-attribute_composition-05] illegal nFields(9)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i<nTuples; i++)
  {
    StAttributeComposition_t stBuff;
    stBuff.attribute_id = atoi(PQgetvalue(res, i, 0));
    stBuff.scope_level = atoi(PQgetvalue(res, i, 1));
    stBuff.composition_number = atoi(PQgetvalue(res, i, 2));
    stBuff.sequence_number = atoi(PQgetvalue(res, i, 3));
    stBuff.parent_scope_level = atoi(PQgetvalue(res, i, 4));
    stBuff.parent_comp_number = atoi(PQgetvalue(res, i, 5));
    stBuff.parent_seq_number = atoi(PQgetvalue(res, i, 6));
    stBuff.av_attr_value = std::string(PQgetvalue(res, i, 7));
    stBuff.av_type_code = getAttributeCode(PQgetvalue(res, i, 8));
#endif

    if(stBuff.av_type_code == A_NONE)
    {
      std::cerr << "WARNING:[readDmpData-attribute_composition-06] illegal AttributeCode" << std::endl;
    }
    else
    {
      MpVcAttributeComposition_t::iterator it = mMpVcAttributeComposition.find(stBuff.attribute_id);
      if(it != mMpVcAttributeComposition.end())
      {
        // 既存レコードに追加
        it->second.push_back(stBuff);
      }
      else
      {
        // 新規レコードの追加
        std::vector<StAttributeComposition_t> vcBuff;
        vcBuff.push_back(stBuff);
        std::pair<MpVcAttributeComposition_t::iterator, bool> result = mMpVcAttributeComposition.insert
          (std::make_pair(stBuff.attribute_id, vcBuff));
        if(!result.second)
        {
          std::cerr << "ERROR:[readDmpData-attribute_composition-07] mMpMpAttributeComposition.insert()" << std::endl;
          goto readDmpData_error;
        }
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] attribute_composition" << std::endl;

  // attribute_group (使用方法不明)
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.attribute_group");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-attribute_group-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-attribute_group-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-attribute_group-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StAttributeGroup_t stBuff;
    stBuff.attribute_id = rs.getInt(0);
#else
  res = PQexec(conn, "SELECT * FROM coi.attribute_group");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-attribute_group-04] PQexec(attribute_group), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 1)
  {
    std::cerr << "ERROR:[readDmpData-attribute_group-05] illegal nFields(1)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StAttributeGroup_t stBuff;
    stBuff.attribute_id = atoi(PQgetvalue(res, i, 0));
#endif

    mVcAttributeGroup.push_back(stBuff);
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] attribute_group" << std::endl;

  // cmp_feat_part_attr (未使用)
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.cmp_feat_part_attr");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StCmpFeatPartAttr_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);
    stBuff.feature_id = rs.getInt(1);
    stBuff.sequence_number = rs.getInt(2);
    stBuff.attribute_id = rs.getInt(3);
#else
  res = PQexec(conn, "SELECT * FROM coi.cmp_feat_part_attr");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-04] PQexec(cmp_feat_part_attr), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 4)
  {
    std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-05] illegal nFields(4)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StCmpFeatPartAttr_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.sequence_number = atoi(PQgetvalue(res, i, 2));
    stBuff.attribute_id = atoi(PQgetvalue(res, i, 3));
#endif

    if((stBuff.feat_category_num < 1) || (stBuff.feat_category_num > 4))
    {
      std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-06] illegal feat_category_num="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    std::map<int, std::vector<StCmpFeatPartAttr_t> >::iterator it = mVcMpVcCmpFeatPartAttr[stBuff.feat_category_num - 1].find(stBuff.feature_id);
    if(it != mVcMpVcCmpFeatPartAttr[stBuff.feat_category_num - 1].end())
    {
      it->second.push_back(stBuff);
    }
    else
    {
      std::vector<StCmpFeatPartAttr_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<std::map<int, std::vector<StCmpFeatPartAttr_t> >::iterator, bool> result =
        mVcMpVcCmpFeatPartAttr[stBuff.feat_category_num - 1].insert(std::make_pair(stBuff.feature_id, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-cmp_feat_part_attr-07] mVcMpVcCmpFeatPartAttr[" << (stBuff.feat_category_num - 1) << "].insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] cmp_feat_part_attr" << std::endl;

  // feature_attr
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.feature_attr");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-feature_attr-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-feature_attr-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-feature_attr-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StFeatureAttr_t stBuff;
    stBuff.feature_category_num = rs.getInt(0);
    stBuff.feature_id = rs.getInt(1);
    stBuff.attribute_id = rs.getInt(2);
#else
  res = PQexec(conn, "SELECT * FROM coi.feature_attr");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-feature_attr-04] PQexec(feature_attr), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 3)
  {
    std::cerr << "ERROR:[readDmpData-feature_attr-05] illegal nFields(3)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StFeatureAttr_t stBuff;
    stBuff.feature_category_num = atoi(PQgetvalue(res, i, 0));
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.attribute_id = atoi(PQgetvalue(res, i, 2));
#endif

    if((stBuff.feature_category_num < 1) || (stBuff.feature_category_num > 4))
    {
      std::cerr << "ERROR:[readDmpData-feature_attr-06] illegal feature_category_num="
                << stBuff.feature_category_num << std::endl;
      goto readDmpData_error;
    }

    std::pair<std::map<int, StFeatureAttr_t>::iterator, bool> result =
      mVcMpFeatureAttr[stBuff.feature_category_num - 1].insert(std::make_pair(stBuff.feature_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-feature_attr-07] mVcMpVcFeatureAttr[" << (stBuff.feature_category_num - 1) << "].insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] feature_attr" << std::endl;

  // feature_category (使わない)
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.feature_category");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-feature_category-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-feature_category-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-feature_category-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StFeatureCategory_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);
    stBuff.feat_category_name = rs.getString(1);
#else
  res = PQexec(conn, "SELECT * FROM coi.feature_category");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-feature_category-04] PQexec(feature_category), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 2)
  {
    std::cerr << "ERROR:[readDmpData-feature_category-05] illegal nFields(2)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StFeatureCategory_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));
    stBuff.feat_category_name = PQgetvalue(res, i, 1);
#endif

    std::pair<MpFeatureCategory_t::iterator, bool> result =
      mMpFeatureCategory.insert(std::make_pair(stBuff.feat_category_num, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-feature_category-06] mMpFeatureCategory.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] feature_category" << std::endl;

  // feature_class_code
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.feature_class_code");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-feature_class_code-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-feature_class_code-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-feature_class_code-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StFeatureClassCode_t stBuff;
    stBuff.feature_class_code = getFeatureClassCode(rs.getString(0).c_str());
    stBuff.description = rs.getString(1);
#else
  res = PQexec(conn, "SELECT * FROM coi.feature_class_code");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-feature_class_code-04] PQexec(feature_class_code), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 2)
  {
    std::cerr << "ERROR:[readDmpData-feature_class_code-05] illegal nFields(2)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StFeatureClassCode_t stBuff;
    stBuff.feature_class_code = getFeatureClassCode(PQgetvalue(res, i, 0));
    stBuff.description = PQgetvalue(res, i, 1);
#endif

    std::pair<MpFeatureClassCode_t::iterator, bool> result =
      mMpFeatureClassCode.insert(std::make_pair(stBuff.feature_class_code, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-feature_class_code-06] mMpFeatureClassCode.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] feature_class_code" << std::endl;

  // pf_area_feature
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_area_feature");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_feature-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_feature-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_feature-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfAreaFeature_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);  // =3
    stBuff.feature_id = rs.getInt(1);
    stBuff.feature_class_code = getFeatureClassCode(rs.getString(2).c_str());
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_area_feature");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_feature-04] PQexec(pf_area_feature), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 3)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_feature-05] illegal nFields(3)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StPfAreaFeature_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));  // =3
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_class_code = getFeatureClassCode(PQgetvalue(res, i, 2));
#endif

    if(stBuff.feat_category_num != 3)
    {
      std::cerr << "ERROR:[readDmpData-pf_area_feature-06] illegal feat_category_num(3)="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    std::pair<MpPfAreaFeature_t::iterator, bool> result =
      mMpPfAreaFeature.insert(std::make_pair(stBuff.feature_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-pf_area_feature-07] mMpPfAreaFeature.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_area_feature" << std::endl;

  // pf_area_topo_prim
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_area_topo_prim");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfAreaTopoPrim_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);  // =3
    stBuff.feature_id = rs.getInt(1);
    stBuff.face_id = rs.getInt(2);
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_area_topo_prim");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-04] PQexec(pf_area_topo_prim), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 3)
  {
    std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-05] illegal nFields(3)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StPfAreaTopoPrim_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));  // =3
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.face_id = atoi(PQgetvalue(res, i, 2));
#endif

    if(stBuff.feat_category_num != 3)
    {
      std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-06] illegal feat_category_num(3)="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    std::pair<MpPfAreaTopoPrim_t::iterator, bool> result =
      mMpPfAreaTopoPrim.insert(std::make_pair(stBuff.feature_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-pf_area_topo_prim-07] mMpPfAreaTopoPrim.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_area_topo_prim" << std::endl;

  // pf_comp_feat_part
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_comp_feat_part");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfCompFeatPart_t stBuff;
    stBuff.comp_feat_category = rs.getInt(0);  // =4
    stBuff.comp_feature_id = rs.getInt(1);
    stBuff.feature_number = rs.getInt(2);
    stBuff.feature_category_num = rs.getInt(3);
    stBuff.feature_id = rs.getInt(4);
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_comp_feat_part");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-04] PQexec(pf_comp_feat_part), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 5)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-05] illegal nFields(5)=" << nFields<< std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i<nTuples; i++)
  {
    StPfCompFeatPart_t stBuff;
    stBuff.comp_feat_category = atoi(PQgetvalue(res, i, 0));  // =4
    stBuff.comp_feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_number = atoi(PQgetvalue(res, i, 2));
    stBuff.feature_category_num = atoi(PQgetvalue(res, i, 3));
    stBuff.feature_id = atoi(PQgetvalue(res, i, 4));
#endif

    if(stBuff.comp_feat_category != 4)
    {
      std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-06] illegal comp_feat_category(4)="
                << stBuff.comp_feat_category << std::endl;
      goto readDmpData_error;
    }

    MpVcPfCompFeatPart_t::iterator it = mMpVcPfCompFeatPart.find(stBuff.comp_feature_id);
    if(it != mMpVcPfCompFeatPart.end())
    {
      it->second.push_back(stBuff);
    }
    else
    {
      std::vector<StPfCompFeatPart_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<MpVcPfCompFeatPart_t::iterator, bool> result =
        mMpVcPfCompFeatPart.insert(std::make_pair(stBuff.comp_feature_id, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-pf_comp_feat_part-07] mMpVcPfCompFeatPart.insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_comp_feat_part" << std::endl;

  // pf_comp_feature
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_comp_feature");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feature-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feature-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feature-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfCompFeature_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);  // =4
    stBuff.feature_id = rs.getInt(1);
    stBuff.feature_class_code = getFeatureClassCode(rs.getString(2).c_str());
    stBuff.from_feat_category = rs.getInt(3);
    stBuff.from_feature_id = rs.getInt(4);
    stBuff.to_feat_category = rs.getInt(5);
    stBuff.to_feature_id = rs.getInt(6);
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_comp_feature");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feature-04] PQexec(pf_comp_feature), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 7)
  {
    std::cerr << "ERROR:[readDmpData-pf_comp_feature-05] illegal nFields(7)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StPfCompFeature_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));  // =4
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_class_code = getFeatureClassCode(PQgetvalue(res, i, 2));
    stBuff.from_feat_category = atoi(PQgetvalue(res, i, 3));
    stBuff.from_feature_id = atoi(PQgetvalue(res, i, 4));
    stBuff.to_feat_category = atoi(PQgetvalue(res, i, 5));
    stBuff.to_feature_id = atoi(PQgetvalue(res, i, 6));
#endif

    if(stBuff.feat_category_num != 4)
    {
      std::cerr << "ERROR:[readDmpData-pf_comp_feature-06] illegal feat_category_num(4)="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    std::pair<MpPfCompFeature_t::iterator, bool> result =
      mMpPfCompFeature.insert(std::make_pair(stBuff.feature_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-pf_comp_feature-07] mMpVcPfCompFeature.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_comp_feature" << std::endl;

  // pf_line_feature
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_line_feature");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_feature-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_feature-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_feature-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfLineFeature_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);  // =2
    stBuff.feature_id = rs.getInt(1);
    stBuff.feature_class_code = getFeatureClassCode(rs.getString(2).c_str());
    stBuff.end_elevation = rs.getInt(3);
    stBuff.from_feat_category = rs.getInt(4);
    stBuff.from_feat_id = rs.getInt(5);
    stBuff.to_feat_category = rs.getInt(6);
    stBuff.to_feat_id = rs.getInt(7);
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_line_feature");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_feature-04] PQexec(pf_line_feature), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 8)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_feature-05] illegal nFields(8)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StPfLineFeature_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));  // =2
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_class_code = getFeatureClassCode(PQgetvalue(res, i, 2));
    stBuff.end_elevation = atoi(PQgetvalue(res, i, 3));
    stBuff.from_feat_category = atoi(PQgetvalue(res, i, 4));
    stBuff.from_feat_id = atoi(PQgetvalue(res, i, 5));
    stBuff.to_feat_category = atoi(PQgetvalue(res, i, 6));
    stBuff.to_feat_id = atoi(PQgetvalue(res, i, 7));
#endif

    if(stBuff.feat_category_num != 2)
    {
      std::cerr << "ERROR:[readDmpData-pf_line_feature-06] illegal feat_category_num(2)="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    std::pair<MpPfLineFeature_t::iterator, bool> result =
      mMpPfLineFeature.insert(std::make_pair(stBuff.feature_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-pf_line_feature-07] mMpPfLineFeature.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_line_feature" << std::endl;

  // pf_line_topo_prim
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_line_topo_prim");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfLineTopoPrim_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);  // =2
    stBuff.feature_id = rs.getInt(1);
    stBuff.sequence_number = rs.getInt(2);
    stBuff.edge_id = rs.getInt(3);
    stBuff.edge_orientation = rs.getInt(4);
    stBuff.start_elevation = rs.getInt(5);
    stBuff.intermitted_elevation = rs.getInt(6);
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_line_topo_prim");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-04] PQexec(pf_line_topo_prim), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 7)
  {
    std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-05] illegal nFields(7)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StPfLineTopoPrim_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));  // =2
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.sequence_number = atoi(PQgetvalue(res, i, 2));
    stBuff.edge_id = atoi(PQgetvalue(res, i, 3));
    stBuff.edge_orientation = atoi(PQgetvalue(res, i, 4));
    stBuff.start_elevation = atoi(PQgetvalue(res, i, 5));
    stBuff.intermitted_elevation = atoi(PQgetvalue(res, i, 6));
#endif

    if(stBuff.feat_category_num != 2)
    {
      std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-06] illegal feat_category_num(2)]="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    MpVcPfLineTopoPrim_t::iterator it = mMpVcPfLineTopoPrim.find(stBuff.feature_id);
    if(it != mMpVcPfLineTopoPrim.end())
    {
      it->second.push_back(stBuff);
    }
    else
    {
      std::vector<StPfLineTopoPrim_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<MpVcPfLineTopoPrim_t::iterator, bool> result =
        mMpVcPfLineTopoPrim.insert(std::make_pair(stBuff.feature_id, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-pf_line_topo_prim-07] mMpVcPfLineTopoPrim.insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_line_topo_prim" << std::endl;

  // pf_point_feature
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.pf_point_feature");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-pf_point_feature-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-pf_point_feature-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-pf_point_feature-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StPfPointFeature_t stBuff;
    stBuff.feat_category_num = rs.getInt(0);  // =1
    stBuff.feature_id = rs.getInt(1);
    stBuff.feature_class_code = getFeatureClassCode(rs.getString(2).c_str());
    stBuff.node_id = rs.getInt(3);
#else
  res = PQexec(conn, "SELECT * FROM coi.pf_point_feature");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-pf_point_feature-04] PQexec(pf_point_feature), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 4)
  {
    std::cerr << "ERROR:[readDmpData-pf_point_feature-05] illegal nFields(4)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StPfPointFeature_t stBuff;
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 0));  // =1
    stBuff.feature_id = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_class_code = getFeatureClassCode(PQgetvalue(res, i, 2));
    stBuff.node_id = atoi(PQgetvalue(res, i, 3));
#endif

    if(stBuff.feat_category_num != 1)
    {
      std::cerr << "ERROR:[readDmpData-pf_point_feature-06] illegal feat_category_num(1)="
                << stBuff.feat_category_num << std::endl;
      goto readDmpData_error;
    }

    std::pair<MpPfPointFeature_t::iterator, bool> result =
      mMpPfPointFeature.insert(std::make_pair(stBuff.feature_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-pf_point_feature-07] mMpPfPointFeature.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] pf_point_feature" << std::endl;

  // relation_feat_attr
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.relation_feat_attr");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-relation_feat_attr-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-relation_feat_attr-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-relation_feat_attr-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StRelationFeatAttr_t stBuff;
    stBuff.relationship_id = rs.getInt(0);
    stBuff.role_number = rs.getInt(1);
    stBuff.feature_number = rs.getInt(2);
    stBuff.attribute_id = rs.getInt(3);
#else
  res = PQexec(conn, "SELECT * FROM coi.relation_feat_attr");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-relation_feat_attr-04] PQexec(relation_feat_attr), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 4)
  {
    std::cerr << "ERROR:[readDmpData-relation_feat_attr-05] illegal nFields(4)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StRelationFeatAttr_t stBuff;
    stBuff.relationship_id = atoi(PQgetvalue(res, i, 0));
    stBuff.role_number = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_number = atoi(PQgetvalue(res, i, 2));
    stBuff.attribute_id = atoi(PQgetvalue(res, i, 3));
#endif

    MpVcRelationFeatAttr_t::iterator it = mMpVcRelationFeatAttr.find(stBuff.relationship_id);
    if(it != mMpVcRelationFeatAttr.end())
    {
      it->second.push_back(stBuff);
    }
    else
    {
      std::vector<StRelationFeatAttr_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<MpVcRelationFeatAttr_t::iterator, bool> result =
        mMpVcRelationFeatAttr.insert(std::make_pair(stBuff.relationship_id, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-relation_feat_attr-06] mMpVcRelationFeatAttr.insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] relation_feat_attr" << std::endl;

  // relation_role
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.relation_role");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-relation_role-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-relation_role-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-relation_role-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StRelationRole_t stBuff;
    stBuff.rel_type_code = getRelationshipType(rs.getString(0).c_str());
    stBuff.role_number = rs.getInt(1);
    stBuff.role_name = rs.getString(2);
    stBuff.repeatable = (*(rs.getString(3).c_str()) == 't');
    stBuff.mandatory = (*(rs.getString(4).c_str()) == 't');
#else
  res = PQexec(conn, "SELECT * FROM coi.relation_role");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-relation_role-04] PQexec(relation_role), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 5)
  {
    std::cerr << "ERROR:[readDmpData-relation_role-05] illegal nFields(5)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StRelationRole_t stBuff;
    stBuff.rel_type_code = getRelationshipType(PQgetvalue(res, i, 0));
    stBuff.role_number = atoi(PQgetvalue(res, i, 1));
    stBuff.role_name = std::string(PQgetvalue(res, i, 2));
    stBuff.repeatable = (*PQgetvalue(res, i, 3) == 't');
    stBuff.mandatory = (*PQgetvalue(res, i, 4) == 't');
#endif

    MpVcRelationRole_t::iterator it = mMpVcRelationRole.find(stBuff.rel_type_code);
    if(it != mMpVcRelationRole.end())
    {
      it->second.push_back(stBuff);
    }
    else
    {
      std::vector<StRelationRole_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<MpVcRelationRole_t::iterator, bool> result =
        mMpVcRelationRole.insert(std::make_pair(stBuff.rel_type_code, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-relation_role-06] mMpVcRelationRole.insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] relation_role" << std::endl;

  // relation_type_code
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.relation_type_code");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-relation_type_code-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-relation_type_code-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-relation_type_code-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StRelationTypeCode_t stBuff;
    stBuff.rel_type_code = getRelationshipType(rs.getString(0).c_str());
    stBuff.description = rs.getString(1);
#else
  res = PQexec(conn, "SELECT * FROM coi.relation_type_code");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-relation_type_code-04] PQexec(relation_type_code), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 2)
  {
    std::cerr << "ERROR:[readDmpData-relation_type_code-05] illegal nFields(2)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StRelationTypeCode_t stBuff;
    stBuff.rel_type_code = getRelationshipType(PQgetvalue(res, i, 0));
    stBuff.description = std::string(PQgetvalue(res, i, 1));
#endif

    std::pair<MpRelationTypeCode_t::iterator, bool> result =
      mMpRelationTypeCode.insert(std::make_pair(stBuff.rel_type_code, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-relation_type_code-06] mMpRelationTypeCode.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] relation_type_code" << std::endl;

  // relationship
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.relationship");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-relationship-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-relationship-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-relationship-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StRelationship_t stBuff;
    stBuff.relationship_id = rs.getInt(0);
    stBuff.rel_type = getRelationshipType(rs.getString(1).c_str());
#else
  res = PQexec(conn, "SELECT * FROM coi.relationship");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-relationship-04] PQexec(relationship), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 2)
  {
    std::cerr << "ERROR:[readDmpData-relationship-05] illegal nFields(2)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StRelationship_t stBuff;
    stBuff.relationship_id = atoi(PQgetvalue(res, i, 0));
    stBuff.rel_type = getRelationshipType(PQgetvalue(res, i, 1));
#endif

    std::pair<MpRelationship_t::iterator, bool> result =
      mMpRelationship.insert(std::make_pair(stBuff.relationship_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-relationship-06] mMpRelationship.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] relationship" << std::endl;

  // relationship_attr
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.relationship_attr");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-relationship_attr-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-relationship_attr-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-relationship_attr-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StRelationshipAttr_t stBuff;
    stBuff.relationship_id = rs.getInt(0);
    stBuff.attribute_id = rs.getInt(1);
#else
  res = PQexec(conn, "SELECT * FROM coi.relationship_attr");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-relationship_attr-04] PQexec(relationship_attr), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 2)
  {
    std::cerr << "ERROR:[readDmpData-relationship_attr-05] illegal nFields(2)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StRelationshipAttr_t stBuff;
    stBuff.relationship_id = atoi(PQgetvalue(res, i, 0));
    stBuff.attribute_id = atoi(PQgetvalue(res, i, 1));
#endif

    MpVcRelationshipAttr_t::iterator it = mMpVcRelationshipAttr.find(stBuff.relationship_id);
    if(it != mMpVcRelationshipAttr.end())
    {
      it->second.push_back(stBuff);
    }
    else
    {
      std::vector<StRelationshipAttr_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<MpVcRelationshipAttr_t::iterator, bool> result =
        mMpVcRelationshipAttr.insert(std::make_pair(stBuff.relationship_id, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-relationship_attr-06] mMpVcRelationshipAttr.insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] relationship_attr" << std::endl;

  // relationship_feat
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT * FROM coi.relationship_feat");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-relationship_feat-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-relationship_feat-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-relationship_feat-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StRelationshipFeat_t stBuff;
    stBuff.relationship_id = rs.getInt(0);
    stBuff.role_number = rs.getInt(1);
    stBuff.feature_number = rs.getInt(2);
    stBuff.feat_category_num = rs.getInt(3);
    stBuff.feature_id = rs.getInt(4);
#else
  res = PQexec(conn, "SELECT * FROM coi.relationship_feat");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-relationship_feat-04] PQexec(relationship_feat), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 5)
  {
    std::cerr << "ERROR:[readDmpData-relationship_feat-05] illegal nFields(5)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StRelationshipFeat_t stBuff;
    stBuff.relationship_id = atoi(PQgetvalue(res, i, 0));
    stBuff.role_number = atoi(PQgetvalue(res, i, 1));
    stBuff.feature_number = atoi(PQgetvalue(res, i, 2));
    stBuff.feat_category_num = atoi(PQgetvalue(res, i, 3));
    stBuff.feature_id = atoi(PQgetvalue(res, i, 4));
#endif

    MpRelationship_t::iterator it = mMpRelationship.find(stBuff.relationship_id);
    if(it == mMpRelationship.end())
    {
      std::cerr << "ERROR:[readDmpData-relationship_feat-06] missing relationship_id=" << stBuff.relationship_id << std::endl;
      goto readDmpData_error;
    }
    stBuff.rel_type = it->second.rel_type;

    MpVcRelationshipFeat_t::iterator itf = mMpVcRelationshipFeat.find(stBuff.relationship_id);
    if(itf != mMpVcRelationshipFeat.end())
    {
      itf->second.push_back(stBuff);
    }
    else
    {
      std::vector<StRelationshipFeat_t> vcBuff;
      vcBuff.push_back(stBuff);
      std::pair<MpVcRelationshipFeat_t::iterator, bool> result =
        mMpVcRelationshipFeat.insert(std::make_pair(stBuff.relationship_id, vcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[readDmpData-relationship_feat-07] mMpVcRelationshipFeat.insert()" << std::endl;
        goto readDmpData_error;
      }
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] relationship_feat" << std::endl;

  // st_edge
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT edge_id, start_node, end_node, next_left_edge, next_right_edge,"
                      " left_face, right_face, ST_AsText(geometry) FROM coi.st_edge");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-st_edge-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-st_edge-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-st_edge-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StStEdge_t stBuff;
    stBuff.edge_id = rs.getInt(0);
    stBuff.start_node = rs.getInt(1);
    stBuff.end_node = rs.getInt(2);
    stBuff.next_left_edge = rs.getInt(3);
    stBuff.next_right_edge = rs.getInt(4);
    stBuff.left_face = rs.getInt(5);
    stBuff.right_face = rs.getInt(6);
    if(!getGeometryLinestringZ(rs.getString(7), sno, stBuff.geometry))
    {
      std::cerr << "ERROR:[readDmpData-st_edge-04] getGeometryLinestringZ()" << std::endl;
      goto readDmpData_error;
    }
#else
  res = PQexec(conn, "SELECT edge_id, start_node, end_node, next_left_edge, next_right_edge,"
                     " left_face, right_face, ST_AsText(geometry) FROM coi.st_edge");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-st_edge-05] PQexec(st_edge), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 8)
  {
    std::cerr << "ERROR:[readDmpData-st_edge-06] illegal nFields(8)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StStEdge_t stBuff;
    stBuff.edge_id = atoi(PQgetvalue(res, i, 0));
    stBuff.start_node = atoi(PQgetvalue(res, i, 1));
    stBuff.end_node = atoi(PQgetvalue(res, i, 2));
    stBuff.next_left_edge = atoi(PQgetvalue(res, i, 3));
    stBuff.next_right_edge = atoi(PQgetvalue(res, i, 4));
    stBuff.left_face = atoi(PQgetvalue(res, i, 5));
    stBuff.right_face = atoi(PQgetvalue(res, i, 6));
    if(!getGeometryLinestringZ(std::string(PQgetvalue(res, i, 7)), sno, stBuff.geometry))
    {
      std::cerr << "ERROR:[readDmpData-st_edge-07] getGeometryLinestringZ()" << std::endl;
      goto readDmpData_error;
    }
#endif

    std::pair<MpStEdge_t::iterator, bool> result =
      mMpStEdge.insert(std::make_pair(stBuff.edge_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-st_edge-08] mMpStEdge.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] st_edge" << std::endl;

  // st_face
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT face_id, ST_AsText(mbr), slope, aspect, light_value, extended_geom_flag FROM coi.st_face");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-st_face-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-st_face-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-st_face-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StStFace_t stBuff;
    stBuff.face_id = rs.getInt(0);
    if(!getGeometryPolygonZ(rs.getString(1), sno, stBuff.mbr))
    {
      std::cerr << "ERROR:[readDmpData-st_face-04] getGeometryPolygonZ()" << std::endl;
      goto readDmpData_error;
    }
    stBuff.slope = rs.getDouble(2);
    stBuff.aspect = rs.getInt(3);
    stBuff.light_value = rs.getInt(4);
    stBuff.extended_geom_flag = (*(rs.getString(5).c_str()) == 't');
#else
  res = PQexec(conn, "SELECT face_id, ST_AsText(mbr), slope, aspect, light_value, extended_geom_flag FROM coi.st_face");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-st_face-05] PQexec(st_st_face), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 6)
  {
    std::cerr << "ERROR:[readDmpData-st_face-06] illegal nFields(6)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StStFace_t stBuff;
    stBuff.face_id = atoi(PQgetvalue(res, i, 0));
    if(!getGeometryPolygonZ(std::string(PQgetvalue(res, i, 1)), sno, stBuff.mbr))
    {
      std::cerr << "ERROR:[readDmpData-st_face-07] getGeometryPolygonZ()" << std::endl;
      goto readDmpData_error;
    }
    stBuff.slope = atof(PQgetvalue(res, i, 2));
    stBuff.aspect = atoi(PQgetvalue(res, i, 3));
    stBuff.light_value = atoi(PQgetvalue(res, i, 4));
    stBuff.extended_geom_flag = (*PQgetvalue(res, i, 5) == 't');
#endif

    std::pair<MpStFace_t::iterator, bool> result =
      mMpStFace.insert(std::make_pair(stBuff.face_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-st_face-08] mMpStFace.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] st_face" << std::endl;

  // st_node
#ifdef USE_DMLIB
  try
  {
    rs = con->execute("SELECT node_id, ST_AsText(geometry), containing_face FROM coi.st_node");
  }
  catch(ConnectionTimeoutException ex1)
  {
    std::cerr << "ERROR:[readDmpData-st_node-01] execute(), ConnectionTimeoutException" << std::endl;
    goto readDmpData_error;
  }
  catch(SQLException ex2)
  {
    std::cerr << "ERROR:[readDmpData-st_node-02] execute(), SQLException" << std::endl;
    goto readDmpData_error;
  }
  catch(...)
  {
    std::cerr << "ERROR:[readDmpData-st_node-03] execute(), Exception" << std::endl;
    goto readDmpData_error;
  }
  while(rs.next())
  {
    StStNode_t stBuff;
    stBuff.node_id= rs.getInt(0);
    if(!getGeometryPointZ(rs.getString(1), sno, stBuff.geometry))
    {
      std::cerr << "ERROR:[readDmpData-st_node-04] getGeometryPointZ()" << std::endl;
      goto readDmpData_error;
    }
    stBuff.containing_face= rs.getInt(2);
#else
  res = PQexec(conn, "SELECT node_id, ST_AsText(geometry), containing_face FROM coi.st_node");
  if(PQresultStatus(res) != PGRES_TUPLES_OK)
  {
    std::cerr << "ERROR:[readDmpData-st_node-05] PQexec(st_node), err="
              << std::string(PQerrorMessage(conn)) << std::endl;
    goto readDmpData_error;
  }

  nFields = PQnfields(res);
  nTuples = PQntuples(res);
  if(nFields != 3)
  {
    std::cerr << "ERROR:[readDmpData-st_node-06] illegal nFields(3)=" << nFields << std::endl;
    goto readDmpData_error;
  }

  for(int i=0; i < nTuples; i++)
  {
    StStNode_t stBuff;
    stBuff.node_id= atoi(PQgetvalue(res, i, 0));
    if(!getGeometryPointZ(std::string(PQgetvalue(res, i, 1)), sno, stBuff.geometry))
    {
      std::cerr << "ERROR:[readDmpData-st_node-07] getGeometryPointZ()" << std::endl;
      goto readDmpData_error;
    }
    stBuff.containing_face= atoi(PQgetvalue(res, i, 2));
#endif

    std::pair<MpStNode_t::iterator, bool> result =
      mMpStNode.insert(std::make_pair(stBuff.node_id, stBuff));
    if(!result.second)
    {
      std::cerr << "ERROR:[readDmpData-st_node-08] mMpStNode.insert()" << std::endl;
      goto readDmpData_error;
    }
  }
#ifdef USE_DMLIB
  rs.close();
#else
  PQclear(res);
#endif
  //std::cerr << "DEBUG:[readDmpData] st_node" << std::endl;

  // 正常終了
#ifdef USE_DMLIB
  con->disconnect();
#else
  PQfinish(conn);
#endif
  return true;

  // 読み込みエラー終了
 readDmpData_error:
#ifdef USE_DMLIB
  rs.close();
  con->disconnect();
#else
  PQclear(res);
  PQfinish(conn);
#endif

  return false;
}

/************************************************************
*  地物データの再構築
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
bool rebuildFeatureData(void)
{
  //std::cerr << "DEBUG:[rebuildFeatureData] START" << std::endl;

  /**************************************
  *  複合(Complex)データの読み込みと解析
  **************************************/

  // 複合データ毎の読み込み処理
  for(MpPfCompFeature_t::const_iterator it = mMpPfCompFeature.begin();
      it != mMpPfCompFeature.end(); ++it)
  {
    if(it->second.feature_class_code == FC8110_Lane)
    {
      //-----------------------
      // [FC8110]レーン本体
      //-----------------------
      StDmpLane_t stLnBuff;
      stLnBuff.feature_id = it->second.feature_id;  // 地物ID (キー)
      stLnBuff.pcfp1_feature_id = 0;
      stLnBuff.pltp1_edge_id = 0;
      stLnBuff.pcfp2_feature_id = 0;
      stLnBuff.patp2_face_id = 0;
      stLnBuff.dA_LR = 0;
      stLnBuff.dA_WI = 0;
      stLnBuff.iA_EV = 0;
      stLnBuff.iA_TR = 0;
      stLnBuff.iA_VT = 0;
      stLnBuff.dA_YN = 0;
      stLnBuff.dA_PN = 0;
      stLnBuff.dA_WR = 0;
      stLnBuff.dA_WL = 0;
      stLnBuff.iA_NL = 0;
      stLnBuff.iA_OY = 0;
      stLnBuff.iA_SP = 0;

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stLnBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Lane-01] illegal comp feature_id="
                  << stLnBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        if(itpv->feature_category_num == CAT_Line)
        {
          // [FL8130]レーン中心線/走行目安線のはず
          stLnBuff.pcfp1_feature_id = itpv->feature_id;
          mCmpDoneLine.insert(stLnBuff.pcfp1_feature_id);  // 処理済みに追加

          // pf_line_feature の検索
          MpPfLineFeature_t::const_iterator itlf = mMpPfLineFeature.find(stLnBuff.pcfp1_feature_id);
          if(itlf == mMpPfLineFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-02] illegal pcfp1_feature_id="
                      << stLnBuff.pcfp1_feature_id << std::endl;
            return false;
          }
          // レーン中心線/走行目安線であることの確認
          if(itlf->second.feature_class_code != FL8130_LaneLine)
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-03] illegal pcfp1_feature_id="
                      << stLnBuff.pcfp1_feature_id << std::endl;
            return false;
          }

          // pf_line_topo_prim の検索
          MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stLnBuff.pcfp1_feature_id);
          if(itlt == mMpVcPfLineTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-04] illegal pcfp1_feature_id="
                      << stLnBuff.pcfp1_feature_id << std::endl;
            return false;
          }
          if(itlt->second.size() != 1)
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-05] illegal size="
                      << itlt->second.size() << std::endl;
            return false;
          }
          stLnBuff.pltp1_edge_id = itlt->second[0].edge_id;

          // st_edge の検索
          MpStEdge_t::const_iterator itle = mMpStEdge.find(stLnBuff.pltp1_edge_id);
          if(itle == mMpStEdge.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-06] illegal pltp1_edge_id="
                      << stLnBuff.pltp1_edge_id << std::endl;
            return false;
          }
          stLnBuff.se1_geometry  = itle->second.geometry;

        }
        else if(itpv->feature_category_num == CAT_Area)
        {
          // [FA8120]レーン走行可能領域のはず
          stLnBuff.pcfp2_feature_id = itpv->feature_id;
          mCmpDoneArea.insert(stLnBuff.pcfp2_feature_id);  // 処理済みに追加

          // pf_area_feature の検索
          MpPfAreaFeature_t::const_iterator itaf = mMpPfAreaFeature.find(stLnBuff.pcfp2_feature_id);
          if(itaf == mMpPfAreaFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-07] illegal pcfp2_feature_id="
                      << stLnBuff.pcfp2_feature_id << std::endl;
            return false;
          }
          // レーン走行可能領域であることの確認
          if(itaf->second.feature_class_code != FA8120_LaneArea)
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-08] illegal pcfp2_feature_id="
                      << stLnBuff.pcfp2_feature_id << std::endl;
            return false;
          }

          // pf_area_topo_prim の検索
          MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stLnBuff.pcfp2_feature_id);
          if(itat == mMpPfAreaTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-09] illegal pcfp2_feature_id="
                      << stLnBuff.pcfp2_feature_id << std::endl;
            return false;
          }
          stLnBuff.patp2_face_id = itat->second.face_id;

          // st_face の検索
          MpStFace_t::const_iterator itasf = mMpStFace.find(stLnBuff.patp2_face_id);
          if(itasf == mMpStFace.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-10] illegal patp2_face_id="
                      << stLnBuff.patp2_face_id << std::endl;
            return false;
          }
          stLnBuff.sf2_mbr = itasf->second.mbr;
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-Lane-11] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
      }

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Complex, stLnBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Lane-12] missing attribute, feature_id="
                  << stLnBuff.feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_EV)
        {
          // Lane Type (0:normal lane, 1:emergency lane, ...)
          stLnBuff.iA_EV = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_LR)
        {
          // Length of Road Element (数値)
          stLnBuff.dA_LR = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_NL)
        {
          // Number of Lane (レーン数(交差点内は0))
          stLnBuff.iA_NL = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_OY)
        {
          // レーンの走行方向(0:順方向, 1:逆方向) 基本的には0
          stLnBuff.iA_OY = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_PN)
        {
          // pitch angle (数値)
          stLnBuff.dA_PN = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_SP)
        {
          // Speed (走行時の想定速度)
          stLnBuff.iA_SP = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_TR)
        {
          // Toll Road (0:False, True)
          stLnBuff.iA_TR = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_VT)
        {
          // Vehicle Type (0:all, 11:Passenger cars, 15:emergency vehicle, 16:taxi, 24:bicycle, 25:pedestrian, ...)
          stLnBuff.iA_VT = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_WI)
        {
          // Width (数値)
          stLnBuff.dA_WI = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_WL)
        {
          // Width of Leftside (左幅員, 数値)
          stLnBuff.dA_WL = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_WR)
        {
          // Width of Rightside (右幅員, 数値)
          stLnBuff.dA_WR = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_YN)
        {
         // yaw angle (数値)
          stLnBuff.dA_YN = atof(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-Lane-13] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // Laneデータの登録
      std::pair<MpDmpLane_t::iterator, bool> result =
        g_MpDmpLane.insert(std::make_pair(stLnBuff.feature_id, stLnBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Lane-14] g_MpDmpLane.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FC8140_ExtendedPedestrian)
    {
      //-----------------------
      // [FC8140]歩道(ExtendedPedestrian)
      //-----------------------
      StDmpExtendedPedestrian_t stEpBuff;
      stEpBuff.feature_id = it->second.feature_id;  // 地物ID (キー)

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stEpBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-01] illegal feature_id="
                  << stEpBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        if(itpv->feature_category_num == CAT_Line)
        {
          // [FL8160]歩道／横断歩道中心線のはず
          StDmpPedestrianLine_t stPlBuff;
          stPlBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
          stPlBuff.pltp_edge_id = 0;
          mCmpDoneLine.insert(itpv->feature_id);  // 処理済みに追加

          // pf_line_feature の検索
          MpPfLineFeature_t::const_iterator itlf = mMpPfLineFeature.find(stPlBuff.feature_id);
          if(itlf == mMpPfLineFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-02] illegal feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }
          // 歩道／横断歩道中心線であることの確認
          if(itlf->second.feature_class_code != FL8160_PedestrianLine)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-03] illegal feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }

          // pf_line_topo_prim の検索
          MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stPlBuff.feature_id);
          if(itlt == mMpVcPfLineTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-04] illegal feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }
          if(itlt->second.size() != 1)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-05] illegal size="
                      << itlt->second.size() << std::endl;
            return false;
          }
          stPlBuff.pltp_edge_id = itlt->second[0].edge_id;

          // st_edge の検索
          MpStEdge_t::const_iterator itle = mMpStEdge.find(stPlBuff.pltp_edge_id);
          if(itle == mMpStEdge.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-06] illegal pltp_edge_id="
                      << stPlBuff.pltp_edge_id << std::endl;
            return false;
          }
          stPlBuff.se_geometry  = itle->second.geometry;  // 点の系列

          // 個別の歩道／横断歩道中心線の保存
          stEpBuff.mVcDmpPedestrianLine.push_back(stPlBuff);
        }
        else if(itpv->feature_category_num == CAT_Area)
        {
          // [FA8150]歩道／横断歩道(縞模様の外形)形状のはず
          StDmpPedestrianArea_t stPaBuff;
          stPaBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
          stPaBuff.patp_face_id = 0;
          mCmpDoneArea.insert(itpv->feature_id);  // 処理済みに追加

          // pf_area_feature の検索
          MpPfAreaFeature_t::const_iterator itaf = mMpPfAreaFeature.find(stPaBuff.feature_id);
          if(itaf == mMpPfAreaFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-07] illegal feature_id="
                      << stPaBuff.feature_id << std::endl;
            return false;
          }
          // 歩道／横断歩道形状であることの確認
          if(itaf->second.feature_class_code != FA8150_PedestrianArea)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-08] feature_class_code ="
                      << itaf->second.feature_class_code << std::endl;
            return false;
          }

          // pf_area_topo_prim の検索
          MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stPaBuff.feature_id);
          if(itat == mMpPfAreaTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-09] illegal feature_id="
                      << stPaBuff.feature_id << std::endl;
            return false;
          }
          stPaBuff.patp_face_id = itat->second.face_id;  // st_faceへの参照キー

          // st_face の検索
          MpStFace_t::const_iterator itasf = mMpStFace.find(stPaBuff.patp_face_id);
          if(itasf == mMpStFace.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-10] illegal patp_face_id="
                      << stPaBuff.patp_face_id << std::endl;
            return false;
          }
          stPaBuff.sf_mbr = itasf->second.mbr;  // 領域を表す多角形

          // 個別の歩道／横断歩道形状の保存
          stEpBuff.mVcDmpPedestrianArea.push_back(stPaBuff);
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-11] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
      }

      // 歩道の登録
      std::pair<MpDmpExtendedPedestrian_t::iterator, bool> result =
        g_MpDmpExtendedPedestrian.insert(std::make_pair(stEpBuff.feature_id, stEpBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-12] g_ExtendedPedestrian.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FC8145_ExtendedPedestrianCrossing)
    {
      //-----------------------
      // [FC8145]横断歩道(ExtendedPedestrianCrossing)
      //-----------------------
      StDmpExtendedPedestrianCrossing_t stEpcBuff;
      stEpcBuff.feature_id = it->second.feature_id;  // 地物ID (キー)

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stEpcBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-01] illegal feature_id="
                  << stEpcBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        if(itpv->feature_category_num == CAT_Line)
        {
          // [FL8160]歩道／横断歩道中心線のはず
          StDmpPedestrianLine_t stPlBuff;
          stPlBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
          stPlBuff.pltp_edge_id = 0;
          mCmpDoneLine.insert(itpv->feature_id);  // 処理済みに追加

          // pf_line_feature の検索
          MpPfLineFeature_t::const_iterator itlf = mMpPfLineFeature.find(stPlBuff.feature_id);
          if(itlf == mMpPfLineFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-02] illegal feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }
          // 歩道／横断歩道中心線であることの確認
          if(itlf->second.feature_class_code != FL8160_PedestrianLine)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-03] illegal feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }

          // pf_line_topo_prim の検索
          MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stPlBuff.feature_id);
          if(itlt == mMpVcPfLineTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-04] illegal feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }
          if(itlt->second.size() != 1)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-05] illegal size="
                      << itlt->second.size() << std::endl;
            return false;
          }
          stPlBuff.pltp_edge_id = itlt->second[0].edge_id;

          // st_edge の検索
          MpStEdge_t::const_iterator itle = mMpStEdge.find(stPlBuff.pltp_edge_id);
          if(itle == mMpStEdge.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-06] illegal pltp_edge_id="
                      << stPlBuff.pltp_edge_id << std::endl;
            return false;
          }
          stPlBuff.se_geometry  = itle->second.geometry;  // 点の系列

          // 個別の歩道／横断歩道中心線の保存
          stEpcBuff.mVcDmpPedestrianLine.push_back(stPlBuff);
        }
        else if(itpv->feature_category_num == CAT_Area)
        {
          mCmpDoneArea.insert(itpv->feature_id);  // 処理済みに追加

          // pf_area_feature の検索
          MpPfAreaFeature_t::const_iterator itaf = mMpPfAreaFeature.find(itpv->feature_id);
          if(itaf == mMpPfAreaFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-07] illegal feature_id="
                      << itpv->feature_id << std::endl;
            return false;
          }

          // pf_area_topo_prim の検索
          MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(itpv->feature_id);
          if(itat == mMpPfAreaTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-08] illegal feature_id="
                      << itpv->feature_id << std::endl;
            return false;
          }

          // st_face の検索
          MpStFace_t::const_iterator itasf = mMpStFace.find(itat->second.face_id);
          if(itasf == mMpStFace.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrian-09] illegal patp_face_id="
                      << itat->second.face_id << std::endl;
            return false;
          }

          // 個別部品(子)地物毎の処理
          if(itaf->second.feature_class_code == FA8150_PedestrianArea)
          {
            // [FA8150]歩道／横断歩道(縞模様の外形)形状
            StDmpPedestrianArea_t stPaBuff;
            stPaBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
            stPaBuff.patp_face_id = itat->second.face_id;
            stPaBuff.sf_mbr = itasf->second.mbr;  // 領域を表す多角形

            // 個別の歩道／横断歩道形状の保存
            stEpcBuff.mVcDmpPedestrianArea.push_back(stPaBuff);
          }
          else if(itaf->second.feature_class_code == FA8252_PedestrianCrossingMarkingShape)
          {
            // [FA8252]横断歩道形状(縞模様毎)
            StDmpPedestrianCrossingMarkingsShape_t stPcmsBuff;
            stPcmsBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
            stPcmsBuff.patp_face_id = itat->second.face_id;
            stPcmsBuff.sf_mbr = itasf->second.mbr;  // 領域を表す多角形

            // 個別の歩道／横断歩道形状の保存
            stEpcBuff.mVcDmpPedestrianCrossingMarkingsShape.push_back(stPcmsBuff);
          }
          else
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-10] feature_class_code ="
                      << itaf->second.feature_class_code << std::endl;
            return false;
          }

        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-11] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
      }

      // 横断歩道の登録
      std::pair<MpDmpExtendedPedestrianCrossing_t::iterator, bool> result =
        g_MpDmpExtendedPedestrianCrossing.insert(std::make_pair(stEpcBuff.feature_id, stEpcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedPedestrianCrossing-12] g_ExtendedPedestrianCrossing.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FC8220_ExtendedTrafficSign)
    {
      //-----------------------
      // [FC8220]標識
      //-----------------------
      StDmpExtendedTrafficSign_t stEtsBuff;
      stEtsBuff.feature_id = it->second.feature_id;  // 地物ID (キー)
      stEtsBuff.pcfp2_feature_id = 0;
      stEtsBuff.ppf2_node_id = 0;
      stEtsBuff.iA_CT = 0;
      stEtsBuff.dA_PN = 0;
      stEtsBuff.iA_SY = 0;
      stEtsBuff.iA_TS = 0;
      stEtsBuff.dA_YN = 0;

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stEtsBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-01] illegal feature_id="
                  << stEtsBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        // 部品(子)地物カテゴリ=1:Point　のみ
        if(itpv->feature_category_num != CAT_Point)
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-02] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
        mCmpDonePoint.insert(itpv->feature_id);  // 処理済みに追加

        // pf_point_feature の検索
        MpPfPointFeature_t::const_iterator itpf = mMpPfPointFeature.find(itpv->feature_id);
        if(itpf == mMpPfPointFeature.end())
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-03] illegal feature_id="
                    << itpv->feature_id << std::endl;
          return false;
        }

        // st_node の検索
        MpStNode_t::const_iterator itpn = mMpStNode.find(itpf->second.node_id);
        if(itpn == mMpStNode.end())
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-04] illegal node_id="
                    << itpf->second.node_id << std::endl;
          return false;
        }

        // 個別部品(子)地物毎の処理
        if(itpf->second.feature_class_code == FP8210_Pole)
        {
          // [FP8210]ポール
          StDmpPole_t stPlBuff;
          stPlBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
          stPlBuff.node_id = itpf->second.node_id;  // st_nodeへの参照キー
          stPlBuff.sn_geometry = itpn->second.geometry;  // 座標情報
          stPlBuff.dA_DA = 0;  // Diameter
          stPlBuff.dA_LM = 0;  // Measured Length
          stPlBuff.dA_PN = 0;  // pitch angle
          stPlBuff.dA_YN = 0;  // yaw angle

          // attributeの検索
          VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Point, stPlBuff.feature_id);
          if(atc == NULL)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-05] missing attribute, feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }
          // attribute値の設定
          for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
          {
            if(itav->av_type_code == A_DA)
            {
              // Diameter (数値, メートル)
              stPlBuff.dA_DA = atof(itav->av_attr_value.c_str());
            }
            else if(itav->av_type_code == A_LM)
            {
              // Measured Length (数値, メートル)
              stPlBuff.dA_LM = atof(itav->av_attr_value.c_str());
            }
            else if(itav->av_type_code == A_PN)
            {
              // pitch angle (数値)
              stPlBuff.dA_PN = atof(itav->av_attr_value.c_str());
            }
            else if(itav->av_type_code == A_YN)
            {
              // yaw angle (数値)
              stPlBuff.dA_YN = atof(itav->av_attr_value.c_str());
            }
            else
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-06] illegal av_type_code="
                        << itav->av_type_code << std::endl;
              return false;
            }
          }

          // 個別ポールデータの登録
          stEtsBuff.mVcDmpPole.push_back(stPlBuff);
        }
        else if(itpf->second.feature_class_code == FP7220_TrafficSign)
        {
          // [FP7220]標識
          stEtsBuff.pcfp2_feature_id = itpv->feature_id;
          stEtsBuff.ppf2_node_id = itpf->second.node_id;
          stEtsBuff.sn2_geometry = itpn->second.geometry;
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-07] illegal feature_class_code="
                    << itpf->second.feature_class_code << std::endl;
          return false;
        }
      }

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Complex, stEtsBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-08] missing attribute, feature_id="
                  << stEtsBuff.pcfp2_feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_CT)
        {
          // other textual content of traffic sign
          stEtsBuff.iA_CT = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_PN)
        {
          // pitch angle (数値)
          stEtsBuff.dA_PN = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_SY)
        {
          // Symbol on Traffic Sign (数値)
          stEtsBuff.iA_SY= atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_TS)
        {
          // Traffic sign class (50:優先権, 51:方向, 55:停車禁止, 56:警告, ...)
          stEtsBuff.iA_TS = atoi(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_YN)
        {
          // yaw angle (数値)
          stEtsBuff.dA_YN = atof(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-09] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // 標識データの登録
      std::pair<MpDmpExtendedTrafficSign_t::iterator, bool> result =
        g_MpDmpExtendedTrafficSign.insert(std::make_pair(stEtsBuff.feature_id, stEtsBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficSign-10] g_ExtendedTrafficSign.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FC8230_ExtendedTrafficLight)
    {
      //-----------------------
      // [FC8230]信号
      //-----------------------
      StDmpExtendedTrafficLight_t stEtlBuff;
      stEtlBuff.feature_id = it->second.feature_id;  // 地物ID (キー)
      stEtlBuff.iA_LI = 0;
      stEtlBuff.dA_PN = 0;
      stEtlBuff.dA_YN = 0;

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stEtlBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-01] illegal feature_id="
                  << stEtlBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        // 部品(子)地物カテゴリ=1:Point　のみ
        if(itpv->feature_category_num != CAT_Point)
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-02] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
        mCmpDonePoint.insert(itpv->feature_id);  // 処理済みに追加

        // pf_point_feature の検索
        MpPfPointFeature_t::const_iterator itpf = mMpPfPointFeature.find(itpv->feature_id);
        if(itpf == mMpPfPointFeature.end())
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-03] illegal feature_id="
                    << itpv->feature_id << std::endl;
          return false;
        }

        // st_node の検索
        MpStNode_t::const_iterator itpn = mMpStNode.find(itpf->second.node_id);
        if(itpn == mMpStNode.end())
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-04] illegal node_id="
                    << itpf->second.node_id << std::endl;
          return false;
        }

        // 個別部品(子)地物毎の処理
        if(itpf->second.feature_class_code == FP8210_Pole)
        {
          // [FP8210]ポール
          StDmpPole_t stPlBuff;
          stPlBuff.feature_id = itpv->feature_id;  // 地物ID (キー)
          stPlBuff.node_id = itpf->second.node_id;  // st_nodeへの参照キー
          stPlBuff.sn_geometry = itpn->second.geometry;  // 座標情報
          stPlBuff.dA_LM = 0;  // Measured Length
          stPlBuff.dA_DA = 0;  // Diameter
          stPlBuff.dA_PN = 0;  // pitch angle
          stPlBuff.dA_YN = 0;  // yaw angle

          // attributeの検索
          VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Point, stPlBuff.feature_id);
          if(atc == NULL)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-05] missing attribute, feature_id="
                      << stPlBuff.feature_id << std::endl;
            return false;
          }
          // attribute値の設定
          for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
          {
            if(itav->av_type_code == A_DA)
            {
              // Diameter (数値, メートル)
              stPlBuff.dA_DA = atof(itav->av_attr_value.c_str());
            }
            else if(itav->av_type_code == A_LM)
            {
              // Measured Length (数値, メートル)
              stPlBuff.dA_LM = atof(itav->av_attr_value.c_str());
            }
            else if(itav->av_type_code == A_PN)
            {
              // pitch angle (数値)
              stPlBuff.dA_PN = atof(itav->av_attr_value.c_str());
            }
            else if(itav->av_type_code == A_YN)
            {
              // yaw angle (数値)
              stPlBuff.dA_YN = atof(itav->av_attr_value.c_str());
            }
            else
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-06] illegal av_type_code="
                        << itav->av_type_code << std::endl;
              return false;
            }
          }

          // 個別のポールの保存
          stEtlBuff.mVcDmpPole.push_back(stPlBuff);
        }
        else if(itpf->second.feature_class_code == FP8231_TrafficLightLamp)
        {
          // [FP8231]信号ランプ(子)
          StDmpTrafficLightLamp_t stTllBuff;
          stTllBuff.feature_id = itpv->feature_id;
          stTllBuff.node_id = itpf->second.node_id;
          stTllBuff.sn_geometry = itpn->second.geometry;
          stTllBuff.iA_LY = 0;

          // attributeの検索
          VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Point, stTllBuff.feature_id);
          if(atc == NULL)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-07] missing attribute, feature_id="
                      << stTllBuff.feature_id << std::endl;
            return false;
          }
          // attribute値の設定
          for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
          {
            if(itav->av_type_code == A_LY)
            {
              // Lamp Type (ランプ種別, 1: 赤, 2:青, 3:黄, 4: 左矢印, 5: 直進矢印, 6:右矢印, 9:その他)
              stTllBuff.iA_LY = atoi(itav->av_attr_value.c_str());

              if((stTllBuff.iA_LY < 1) || (stTllBuff.iA_LY > 9))
              {
                std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-08] illegal Lamp Type="
                          << stTllBuff.iA_LY << std::endl;
                return false;
              }
            }
            else
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-09] illegal av_type_code="
                        << itav->av_type_code << std::endl;
              return false;
            }
          }

          // 個別の信号ランプの保存
          stEtlBuff.mVcTrafficLightLamp.push_back(stTllBuff);
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-10] illegal feature_class_code="
                    << itpf->second.feature_class_code << std::endl;
          return false;
        }
      }

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Complex, stEtlBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-11] missing attribute, feature_id="
                  << stEtlBuff.feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_LI)
        {
          // Traffic Light Type (信号種別, 1:車両, 2:歩行者, 3:車両歩行者兼用, 9:その他)
          stEtlBuff.iA_LI = atoi(itav->av_attr_value.c_str());
          if((stEtlBuff.iA_LI < 1) || (stEtlBuff.iA_LI > 9))
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-12] illegal Traffic Light Type="
                      << stEtlBuff.iA_LI << std::endl;
            return false;
          }
        }
        else if(itav->av_type_code == A_PN)
        {
          // pitch angle (数値)
          stEtlBuff.dA_PN = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_YN)
        {
          // yaw angle (数値)
          stEtlBuff.dA_YN = atof(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-13] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // 信号データの登録
      std::pair<MpDmpExtendedTrafficLight_t::iterator, bool> result =
        g_MpDmpExtendedTrafficLight.insert(std::make_pair(stEtlBuff.feature_id, stEtlBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedTrafficLight-14] g_ExtendedTrafficLight.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FC8250_ExtendedRoadMarkings)
    {
      //-----------------------
      // [FC8240]街灯
      //-----------------------
      StDmpExtendedLighting_t stElBuff;
      stElBuff.feature_id = it->second.feature_id;  // 地物ID (キー)
      stElBuff.pcfp1_feature_id = 0;
      stElBuff.pcfp1_feature_id = 0;
      stElBuff.ppf1_node_id = 0;
      stElBuff.pcfp2_feature_id = 0;
      stElBuff.pltp2_edge_id = 0; 
      stElBuff.pcfp3_feature_id = 0;
      stElBuff.ppf3_node_id = 0;
      stElBuff.dA3_DA = 0;
      stElBuff.dA3_LM = 0;
      stElBuff.dA3_PN = 0;
      stElBuff.dA3_YN = 0;

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stElBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-01] illegal comp feature_id="
                  << stElBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        if(itpv->feature_category_num == CAT_Line)
        {
          // [FL8241]街灯ランプ

          // 部品(子)地物カテゴリ=2:Lineなので、街灯ランプ
          stElBuff.pcfp2_feature_id = itpv->feature_id;
          mCmpDoneLine.insert(stElBuff.pcfp2_feature_id);  // 処理済みに追加

          // pf_line_feature の検索
          MpPfLineFeature_t::const_iterator itlf = mMpPfLineFeature.find(stElBuff.pcfp2_feature_id);
          if(itlf == mMpPfLineFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-02] illegal pcfp2_feature_id="
                      << stElBuff.pcfp2_feature_id << std::endl;
            return false;
          }
          // 街灯ランプであることの確認
          if(itlf->second.feature_class_code != FL8241_LightingLamp)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-03] illegal pcfp2_feature_id="
                      << stElBuff.pcfp2_feature_id << std::endl;
            return false;
          }

          // pf_line_topo_prim の検索
          MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stElBuff.pcfp2_feature_id);
          if(itlt == mMpVcPfLineTopoPrim.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-04] illegal pcfp2_feature_id="
                      << stElBuff.pcfp2_feature_id << std::endl;
            return false;
          }
          if(itlt->second.size() != 1)
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-05] illegal size="
                      << itlt->second.size() << std::endl;
            return false;
          }
          stElBuff.pltp2_edge_id = itlt->second[0].edge_id;

          // st_edge の検索
          MpStEdge_t::const_iterator itle = mMpStEdge.find(stElBuff.pltp2_edge_id);
          if(itle == mMpStEdge.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-06] illegal pltp1_edge_id="
                      << stElBuff.pltp2_edge_id << std::endl;
            return false;
          }
          stElBuff.se2_geometry  = itle->second.geometry;
        }
        else if(itpv->feature_category_num == CAT_Point)
        {
          // pf_point_feature の検索
          MpPfPointFeature_t::const_iterator itpf = mMpPfPointFeature.find(itpv->feature_id);
          if(itpf == mMpPfPointFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-07] illegal feature_id="
                      << itpv->feature_id << std::endl;
            return false;
          }

          // st_node の検索
          MpStNode_t::const_iterator itpn = mMpStNode.find(itpf->second.node_id);
          if(itpn == mMpStNode.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-08] illegal node_id="
                      << itpf->second.node_id << std::endl;
            return false;
          }

          // 個別部品(子)地物毎の処理
          if(itpf->second.feature_class_code == FP7252_Lighting)
          {
            // [FP7252]街灯
            stElBuff.pcfp1_feature_id = itpv->feature_id;  // 地物ID (キー)
            stElBuff.ppf1_node_id = itpf->second.node_id;  // st_nodeへの参照キー
            stElBuff.sn1_geometry = itpn->second.geometry;  // 座標情報
          }
          else if(itpf->second.feature_class_code == FP8210_Pole)
          {
            // [FP8210]ポール
            stElBuff.pcfp3_feature_id = itpv->feature_id;  // 地物ID (キー)
            stElBuff.ppf3_node_id = itpf->second.node_id;  // st_nodeへの参照キー
            stElBuff.sn3_geometry = itpn->second.geometry;  // 座標情報
            stElBuff.dA3_DA = 0;
            stElBuff.dA3_LM = 0;
            stElBuff.dA3_PN = 0;
            stElBuff.dA3_YN = 0;

            // attributeの検索
            VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Point, stElBuff.pcfp3_feature_id);
            if(atc == NULL)
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-05] missing attribute, feature_id="
                        << stElBuff.pcfp3_feature_id << std::endl;
              return false;
            }
            // attribute値の設定
            for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
            {
              if(itav->av_type_code == A_DA)
              {
                // Diameter (数値, メートル)
                stElBuff.dA3_DA = atof(itav->av_attr_value.c_str());
              }
              else if(itav->av_type_code == A_LM)
              {
                // Measured Length (数値, メートル)
                stElBuff.dA3_LM = atof(itav->av_attr_value.c_str());
              }
              else if(itav->av_type_code == A_PN)
              {
                // pitch angle (数値)
                stElBuff.dA3_PN = atof(itav->av_attr_value.c_str());
              }
              else if(itav->av_type_code == A_YN)
              {
                // yaw angle (数値)
                stElBuff.dA3_YN = atof(itav->av_attr_value.c_str());
              }
              else
              {
                std::cerr << "ERROR:[rebuildFeatureData-ExtendedLightingt-06] illegal av_type_code="
                          << itav->av_type_code << std::endl;
                return false;
              }
            }
          }
          else
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-10] illegal feature_class_code="
                      << itpf->second.feature_class_code << std::endl;
            return false;
          }
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-10] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
      }

      // 街灯データの登録
      std::pair<MpDmpExtendedLighting_t::iterator, bool> result =
        g_MpDmpExtendedLighting.insert(std::make_pair(stElBuff.feature_id, stElBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedLighting-14] g_ExtendedLighting.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FC8250_ExtendedRoadMarkings)
    {
      //-----------------------
      // [FC8250]路面マーク
      //-----------------------
      StDmpExtendedRoadMarkings_t stErmBuff;
      stErmBuff.feature_id = it->second.feature_id;  // 地物ID (キー)
      stErmBuff.pcfp1_feature_id = 0;
      stErmBuff.ppf1_node_id = 0;

      // pf_comp_feat_part の検索
      MpVcPfCompFeatPart_t::const_iterator itp = mMpVcPfCompFeatPart.find(stErmBuff.feature_id);
      if(itp == mMpVcPfCompFeatPart.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-01] illegal feature_id="
                  << stErmBuff.feature_id << std::endl;
        return false;
      }
      // pf_comp_feat_part の要素毎の解析
      for(std::vector<StPfCompFeatPart_t>::const_iterator itpv = itp->second.begin();
          itpv != itp->second.end(); ++itpv)
      {
        if(itpv->feature_category_num == CAT_Point)
        {
          // [FP7254]路面マーク(中心点)のはず
          mCmpDonePoint.insert(itpv->feature_id);  // 処理済みに追加

          // pf_point_feature の検索
          MpPfPointFeature_t::const_iterator itpf = mMpPfPointFeature.find(itpv->feature_id);
          if(itpf == mMpPfPointFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-02] illegal feature_id="
                      << itpv->feature_id << std::endl;
            return false;
          }

          // 路面マーク(中心点)の確認
          if(itpf->second.feature_class_code == FP7254_RoadMarkings)
          {
            // st_node の検索
            MpStNode_t::const_iterator itpn = mMpStNode.find(itpf->second.node_id);
            if(itpn == mMpStNode.end())
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-03] illegal node_id="
                        << itpf->second.node_id << std::endl;
              return false;
            }

            stErmBuff.pcfp1_feature_id = itpv->feature_id;  // 地物ID (キー);
            stErmBuff.ppf1_node_id = itpf->second.node_id;  // st_nodeへの参照キー
            stErmBuff.sn1_geometry = itpn->second.geometry;  // 座標情報
          }
          else
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-04] illegal feature_class_code="
                      << itpf->second.feature_class_code << std::endl;
            return false;
          }
        }
        else if(itpv->feature_category_num == CAT_Area)
        {
          // [FA8251]路面マーク形状のはず
          mCmpDoneArea.insert(itpv->feature_id);  // 処理済みに追加

          // pf_area_feature の検索
          MpPfAreaFeature_t::const_iterator itaf = mMpPfAreaFeature.find(stErmBuff.feature_id);
          if(itaf == mMpPfAreaFeature.end())
          {
            std::cerr << "ERROR:[rebuildFeatureData-Lane-05] illegal pcfp2_feature_id="
                      << stErmBuff.feature_id << std::endl;
            return false;
          }

          // 路面マーク形状であることの確認
          if(itaf->second.feature_class_code == FA8251_RoadMarkingsShape)
          {
            StDmpRoadMarkingsShape_t stRmsBuff;
            stRmsBuff.feature_id = itpv->feature_id;  // 地物ID

            // pf_area_topo_prim の検索
            MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stRmsBuff.feature_id);
            if(itat == mMpPfAreaTopoPrim.end())
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-06] illegal pcfp2_feature_id="
                        << stRmsBuff.feature_id << std::endl;
              return false;
            }
            stRmsBuff.patp_face_id = itat->second.face_id;  // st_faceへの参照キー

            // st_face の検索
            MpStFace_t::const_iterator itasf = mMpStFace.find(stRmsBuff.patp_face_id);
            if(itasf == mMpStFace.end())
            {
              std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-07] illegal patp_face_id="
                        << stRmsBuff.patp_face_id << std::endl;
              return false;
            }
            stRmsBuff.sf_mbr = itasf->second.mbr;  // 領域を表す多角形

            // 個別の路面マーク形状の保存
            stErmBuff.mVcDmpRoadMarkingsShape.push_back(stRmsBuff);
          }
          else
          {
            std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-08] feature_class_code ="
                      << itaf->second.feature_class_code << std::endl;
            return false;
          }
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-09] illegal feature_category_num="
                    << itpv->feature_category_num << std::endl;
          return false;
        }
      }

      // 路面マークデータの登録
      std::pair<MpDmpExtendedRoadMarkings_t::iterator, bool> result =
        g_MpDmpExtendedRoadMarkings.insert(std::make_pair(stErmBuff.feature_id, stErmBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ExtendedRoadMarkings-10] g_ExtendedRoadMarkings.insert()" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "WARNING:[rebuildFeatureData-Complex-01] illegal comp, feature_class_code="
                << it->second.feature_class_code << ", feature_id="
                << it->second.feature_id << std::endl;
      //return false;
    }
  }

  /**************************************
  *  面(Area)データの読み込みと解析
  **************************************/

  // 面データ毎の読み込み処理
  for(MpPfAreaFeature_t::const_iterator it = mMpPfAreaFeature.begin();
      it != mMpPfAreaFeature.end(); ++it)
  {
    // 処理済みかどうかを確認
    std::set<int>::const_iterator itd = mCmpDoneArea.find(it->first);
    if(itd != mCmpDoneArea.end())
      continue;  // 処理済み

    if(it->second.feature_class_code == FA8120_LaneArea)
    {
      // [FA8120]レーン走行可能領域
      std::cerr << "WARNING:[rebuildFeatureData-LaneArea-01] illegal FA8120_LaneArea data" << std::endl;
    }
    else if(it->second.feature_class_code == FA8170_IntersectionAreaShape)
    {
      //-----------------------
      // [FA8170]交差点領域
      //-----------------------
      StDmpIntersectionAreaShape_t stIasBuff;
      stIasBuff.feature_id = it->second.feature_id;  // 地物ID

      // pf_area_topo_prim の検索
      MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stIasBuff.feature_id);
      if(itat == mMpPfAreaTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-IntersectionAreaShape-01] illegal pcfp2_feature_id="
                  << stIasBuff.feature_id << std::endl;
            return false;
      }
      stIasBuff.patp_face_id = itat->second.face_id;

      // st_face の検索
      MpStFace_t::const_iterator itasf = mMpStFace.find(stIasBuff.patp_face_id);
      if(itasf == mMpStFace.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-IntersectionAreaShape-02] illegal patp2_face_id="
                  << stIasBuff.patp_face_id << std::endl;
        return false;
      }
      stIasBuff.sf_mbr = itasf->second.mbr;

      // 交差点領域データの登録
      std::pair<MpDmpIntersectionAreaShape_t::iterator, bool> result =
        g_MpDmpIntersectionAreaShape.insert(std::make_pair(stIasBuff.feature_id, stIasBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-IntersectionAreaShape-03] g_MpDmpIntersectionAreaShape.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FA8251_RoadMarkingsShape)
    {
      // [FA8251]路面マーク形状
      std::cerr << "WARNING:[rebuildFeatureData-RoadMarkingsShape-01] illegal FA8251_RoadMarkingsShape data" << std::endl;
    }
    else if(it->second.feature_class_code == FA8252_PedestrianCrossingMarkingShape)
    {
      // [FA8252]横断歩道形状(縞模様毎)
      std::cerr << "WARNING:[rebuildFeatureData-PedestrianCrossingMarkingShape-01] illegal FA8252_PedestrianCrossingMarkingShape data" << std::endl;
    }
    else if(it->second.feature_class_code == FA8312_Gutter)
    {
      //-----------------------
      // [FA8312]側溝
      //-----------------------
      StDmpGutter_t stGutterBuff;
      stGutterBuff.feature_id = it->second.feature_id;  // 地物ID
      stGutterBuff.iA_GU = 0;

      // pf_area_topo_prim の検索
      MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stGutterBuff.feature_id);
      if(itat == mMpPfAreaTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Gutter-01] illegal pcfp2_feature_id="
                  << stGutterBuff.feature_id << std::endl;
            return false;
      }
      stGutterBuff.patp_face_id = itat->second.face_id;

      // st_face の検索
      MpStFace_t::const_iterator itasf = mMpStFace.find(stGutterBuff.patp_face_id);
      if(itasf == mMpStFace.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Gutter-02] illegal patp2_face_id="
                  << stGutterBuff.patp_face_id << std::endl;
        return false;
      }
      stGutterBuff.sf_mbr = itasf->second.mbr;

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Area, stGutterBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Gutter-03] missing attribute, feature_id="
                  << stGutterBuff.feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_GU)
        {
          // Gutter type (0:蓋なし, 1:フタあり, 2:グレーチング)
          stGutterBuff.iA_GU = atoi(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-Gutter-04] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // 側溝データの登録
      std::pair<MpDmpGutter_t::iterator, bool> result =
        g_MpDmpGutter.insert(std::make_pair(stGutterBuff.feature_id, stGutterBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Gutter-05] g_MpDmpGutter.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FA8313_GuardRail)
    {
      //-----------------------
      // [FA8313]ガードレール
      //-----------------------
      StDmpGuardRail_t stGrBuff;
      stGrBuff.feature_id = it->second.feature_id;  // 地物ID
      stGrBuff.iA_GR = 0;

      // pf_area_topo_prim の検索
      MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stGrBuff.feature_id);
      if(itat == mMpPfAreaTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-GuardRail-01] illegal pcfp2_feature_id="
                  << stGrBuff.feature_id << std::endl;
            return false;
      }
      stGrBuff.patp_face_id = itat->second.face_id;

      // st_face の検索
      MpStFace_t::const_iterator itasf = mMpStFace.find(stGrBuff.patp_face_id);
      if(itasf == mMpStFace.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-GuardRail-02] illegal patp2_face_id="
                  << stGrBuff.patp_face_id << std::endl;
        return false;
      }
      stGrBuff.sf_mbr = itasf->second.mbr;

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Area, stGrBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-GuardRail-03] missing attribute, feature_id="
                  << stGrBuff.feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_GR)
        {
          // Guard Rail type (0:板羽根, 1:パイプ)
          stGrBuff.iA_GR = atof(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-GuardRail-04] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // ガードレールデータの登録
      std::pair<MpDmpGuardRail_t::iterator, bool> result =
        g_MpDmpGuardRail.insert(std::make_pair(stGrBuff.feature_id, stGrBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-GuardRail-05] g_MpDmpGuardRail.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FA8314_ZebraZone)
    {
      //-----------------------
      // [FA8314]ゼブラゾーン
      //-----------------------
      StDmpZebraZone_t stZzBuff;
      stZzBuff.feature_id = it->second.feature_id;  // 地物ID

      // pf_area_topo_prim の検索
      MpPfAreaTopoPrim_t::const_iterator itat = mMpPfAreaTopoPrim.find(stZzBuff.feature_id);
      if(itat == mMpPfAreaTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ZebraZone-01] illegal pcfp2_feature_id="
                  << stZzBuff.feature_id << std::endl;
            return false;
      }
      stZzBuff.patp_face_id = itat->second.face_id;

      // st_face の検索
      MpStFace_t::const_iterator itasf = mMpStFace.find(stZzBuff.patp_face_id);
      if(itasf == mMpStFace.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-ZebraZone-02] illegal patp2_face_id="
                  << stZzBuff.patp_face_id << std::endl;
        return false;
      }
      stZzBuff.sf_mbr = itasf->second.mbr;

      // ゼブラゾーンデータの登録
      std::pair<MpDmpZebraZone_t::iterator, bool> result =
        g_MpDmpZebraZone.insert(std::make_pair(stZzBuff.feature_id, stZzBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-ZebraZone-03] g_MpDmpZebraZone.insert()" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "WARNING:[rebuildFeatureData-Area-01] illegal area, feature_class_code="
                << it->second.feature_class_code << ", feature_id="
                << it->second.feature_id << std::endl;
      //return false;
    }
  }

  /**************************************
  *  線(Line)データの読み込みと解析
  **************************************/

  // 線データ毎の読み込み処理
  for(MpPfLineFeature_t::const_iterator it = mMpPfLineFeature.begin();
      it != mMpPfLineFeature.end(); ++it)
  {
    // 処理済みかどうかを確認
    std::set<int>::const_iterator itd = mCmpDoneLine.find(it->first);
    if(itd != mCmpDoneLine.end())
      continue;  // 処理済み

    if(it->second.feature_class_code == FL8130_LaneLine)
    {
      // [FL8130]レーン中心線/走行目安線
      std::cerr << "WARNING:[rebuildFeatureData-LaneLine-31] illegal FL8130_LaneLine data" << std::endl;
    }
    else if(it->second.feature_class_code == FL8160_PedestrianLine)
    {
      // [FL8160]歩道／横断歩道中心線
      std::cerr << "WARNING:[rebuildFeatureData-PedestrianLine-31] illegal FL8160_PedestrianLine" << std::endl;
    }
    else if(it->second.feature_class_code == FL8241_LightingLamp)
    {
      // [FL8241]街灯ランプ
      std::cerr << "WARNING:[rebuildFeatureData-Line-31] illegal FL8241_LightingLamp" << std::endl;
    }
    else if(it->second.feature_class_code == FL8260_StopLine)
    {
      //-----------------------
      // [FL8260]停止線
      //-----------------------
      StDmpStopLine_t stSlBuff;
      stSlBuff.feature_id = it->second.feature_id;  // 地物ID

      // pf_line_topo_prim の検索
      MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stSlBuff.feature_id);
      if(itlt == mMpVcPfLineTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Line-32] illegal feature_id="
                  << stSlBuff.feature_id << std::endl;
        return false;
      }
      if(itlt->second.size() != 1)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Line-33] illegal size="
                  << itlt->second.size() << std::endl;
        return false;
      }
      stSlBuff.pltp_edge_id = itlt->second[0].edge_id;

      // st_edge の検索
      MpStEdge_t::const_iterator itle = mMpStEdge.find(stSlBuff.pltp_edge_id);
      if(itle == mMpStEdge.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Line-34] illegal pltp_edge_id="
                  << stSlBuff.pltp_edge_id << std::endl;
        return false;
      }
      stSlBuff.se_geometry = itle->second.geometry;

      // 停止線データの登録
      std::pair<MpDmpStopLine_t::iterator, bool> result =
        g_MpDmpStopLine.insert(std::make_pair(stSlBuff.feature_id, stSlBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-35] g_MpDmpStopLine.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FL8310_RoadEdge)
    {
      //-----------------------
      // [FL8310]道路縁
      //-----------------------
      StDmpRoadEdge_t stRoadEdgeBuff;
      stRoadEdgeBuff.feature_id = it->second.feature_id;  // 地物ID

      // pf_line_topo_prim の検索
      MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stRoadEdgeBuff.feature_id);
      if(itlt == mMpVcPfLineTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-RoadEdge-01] illegal feature_id="
                  << stRoadEdgeBuff.feature_id << std::endl;
        return false;
      }
      if(itlt->second.size() != 1)
      {
        std::cerr << "ERROR:[rebuildFeatureData-RoadEdge-02] illegal size="
                  << itlt->second.size() << std::endl;
        return false;
      }
      stRoadEdgeBuff.pltp_edge_id = itlt->second[0].edge_id;

      // st_edge の検索
      MpStEdge_t::const_iterator itle = mMpStEdge.find(stRoadEdgeBuff.pltp_edge_id);
      if(itle == mMpStEdge.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-RoadEdge-03] illegal pltp_edge_id="
                  << stRoadEdgeBuff.pltp_edge_id << std::endl;
        return false;
      }
      stRoadEdgeBuff.se_geometry = itle->second.geometry;

      // 道路縁データの登録
      std::pair<MpDmpRoadEdge_t::iterator, bool> result =
         g_MpDmpRoadEdge.insert(std::make_pair(stRoadEdgeBuff.feature_id, stRoadEdgeBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-RoadEdge-04]  g_MpDmpRoadEdge.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FL8311_Curb)
    {
      //-----------------------
      // [FL8311]縁石
      //-----------------------
      StDmpCurb_t stCbBuff;
      stCbBuff.feature_id = it->second.feature_id;  // 地物ID
      stCbBuff.dA_HT = 0;
      stCbBuff.dA_WI = 0;

      // pf_line_topo_prim の検索
      MpVcPfLineTopoPrim_t::const_iterator itlt = mMpVcPfLineTopoPrim.find(stCbBuff.feature_id);
      if(itlt == mMpVcPfLineTopoPrim.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Curb-01] illegal feature_id="
                  << stCbBuff.feature_id << std::endl;
        return false;
      }
      if(itlt->second.size() != 1)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Curb-02] illegal size="
                  << itlt->second.size() << std::endl;
        return false;
      }
      stCbBuff.pltp_edge_id = itlt->second[0].edge_id;

      // st_edge の検索
      MpStEdge_t::const_iterator itle = mMpStEdge.find(stCbBuff.pltp_edge_id);
      if(itle == mMpStEdge.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Curb-03] illegal pltp_edge_id="
                  << stCbBuff.pltp_edge_id << std::endl;
        return false;
      }
      stCbBuff.se_geometry = itle->second.geometry;

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Line, stCbBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Curb-04] missing attribute, feature_id="
                  << stCbBuff.feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_HT)
        {
          // Height (数値)
          stCbBuff.dA_HT = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_WI)
        {
          // Width (数値)
          stCbBuff.dA_WI = atof(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-Curb-05] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // 縁石データの登録
      std::pair<MpDmpCurb_t::iterator, bool> result =
         g_MpDmpCurb.insert(std::make_pair(stCbBuff.feature_id, stCbBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Curb-06] g_MpDmpCurb.insert()" << std::endl;
        return false;
      }
    }
    else
    {
      std::cerr << "WARNING:[rebuildFeatureData-Line-01] illegal line, feature_class_code="
                << it->second.feature_class_code << ", feature_id="
                << it->second.feature_id << std::endl;
      //return false;
    }
  }

  /**************************************
  *  点(Point)データの読み込みと解析
  **************************************/

  // 点データ毎の読み込み処理
  for(MpPfPointFeature_t::const_iterator it = mMpPfPointFeature.begin();
      it != mMpPfPointFeature.end(); ++it)
  {
    // 処理済みかどうかを確認
    std::set<int>::const_iterator itd = mCmpDonePoint.find(it->first);
    if(itd != mCmpDonePoint.end())
      continue;  // 処理済み

    if(it->second.feature_class_code == FP4120_Junction)
    {
      //-----------------------
      // [FP4120]ジャンクション
      //-----------------------
      StDmpJunction_t stJcBuff;
      stJcBuff.feature_id = it->second.feature_id;  // 地物ID
      stJcBuff.node_id = it->second.node_id;  // st_nodeへの参照キー

      // st_node の検索
      MpStNode_t::const_iterator itn = mMpStNode.find(it->second.node_id);
      if(itn == mMpStNode.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Junction-37] illegal node_id="
                  << it->second.node_id << std::endl;
        return false;
      }
      stJcBuff.sn_geometry = itn->second.geometry;

      // Junctionデータの登録
      std::pair<MpDmpJunction_t::iterator, bool> result =
        g_MpDmpJunction.insert(std::make_pair(stJcBuff.feature_id, stJcBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Junction-38] g_Junction.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FP7220_TrafficSign)
    {
      // [FP7220]標識
      std::cerr << "WARNING:[rebuildFeatureData-TrafficSign-01] illegal FP7220_TrafficSign data" << std::endl;
    }
    else if(it->second.feature_class_code == FP7252_Lighting)
    {
      // [FP7252]街灯
      std::cerr << "WARNING:[rebuildFeatureData-Lighting-01] illegal FP7252_Lighting data" << std::endl;
    }
    else if(it->second.feature_class_code == FP7254_RoadMarkings)
    {
      // [FP7254]路面マーク中心点
      std::cerr << "WARNING:[rebuildFeatureData-RoadMarkings-01] illegal FP7254_RoadMarkings data" << std::endl;
    }
    else if(it->second.feature_class_code == FP8210_Pole)
    {
      //-----------------------
      // [FP8210]ポール
      //-----------------------
      StDmpPole_t stPlBuff;
      stPlBuff.feature_id = it->second.feature_id;  // 地物ID
      stPlBuff.node_id = it->second.node_id;  // st_nodeへの参照キー
      stPlBuff.dA_DA = 0;
      stPlBuff.dA_LM = 0;
      stPlBuff.dA_PN = 0;
      stPlBuff.dA_YN = 0;

      // st_node の検索
      MpStNode_t::const_iterator itn = mMpStNode.find(it->second.node_id);
      if(itn == mMpStNode.end())
      {
        std::cerr << "ERROR:[rebuildFeatureData-Pole-01] illegal node_id="
                  << it->second.node_id << std::endl;
        return false;
      }
      stPlBuff.sn_geometry = itn->second.geometry;

      // attributeの検索
      VcAttributeComposition_t *atc = getFeatureAttribute(CAT_Point, stPlBuff.feature_id);
      if(atc == NULL)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Pole-02] missing attribute, feature_id="
                  << stPlBuff.feature_id << std::endl;
        return false;
      }
      // attribute値の設定
      for(VcAttributeComposition_t::const_iterator itav = atc->begin(); itav != atc->end(); ++itav)
      {
        if(itav->av_type_code == A_DA)
        {
          // Diameter (数値, メートル)
          stPlBuff.dA_DA = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_LM)
        {
          // Measured Length (数値, メートル)
          stPlBuff.dA_LM = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_PN)
        {
          // pitch angle (数値)
          stPlBuff.dA_PN = atof(itav->av_attr_value.c_str());
        }
        else if(itav->av_type_code == A_YN)
        {
          // yaw angle (数値)
          stPlBuff.dA_YN = atof(itav->av_attr_value.c_str());
        }
        else
        {
          std::cerr << "ERROR:[rebuildFeatureData-Pole-03] illegal av_type_code="
                    << itav->av_type_code << std::endl;
          return false;
        }
      }

      // Poleデータの登録
      std::pair<MpDmpPole_t::iterator, bool> result =
        g_MpDmpPole.insert(std::make_pair(stPlBuff.feature_id, stPlBuff));
      if(!result.second)
      {
        std::cerr << "ERROR:[rebuildFeatureData-Pole-04] g_MpDmpPole.insert()" << std::endl;
        return false;
      }
    }
    else if(it->second.feature_class_code == FP8231_TrafficLightLamp)
    {
      // [FP8231]信号ランプ
      std::cerr << "WARNING:[rebuildFeatureData-TrafficLightLamp-01] illegal FP8231_TrafficLightLamp data" << std::endl;
    }
    else
    {
      std::cerr << "WARNING:[rebuildFeatureData-Point-05] illegal point, feature_class_code="
                << it->second.feature_class_code << ", feature_id="
                << it->second.feature_id << std::endl;
      //return false;
    }
  }

  return true;
}

/************************************************************
*  関連データの再構築
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
bool rebuildRelationshipData(void)
{
  //std::cerr << "DEBUG:[rebuildRelationshipData] START" << std::endl;

  for(MpVcRelationshipFeat_t::const_iterator it = mMpVcRelationshipFeat.begin();
      it != mMpVcRelationshipFeat.end(); ++it)
  {
    StDmpRebRelationship_t stRebRelBuff;
    stRebRelBuff.relationship_id = it->first;
    stRebRelBuff.rel_type = it->second[0].rel_type;
    //std::cerr << "DEBUG:[rebuildRelationshipData] relationship_id="
    //          << stRebRelBuff.relationship_id << ", rel_type="
    //          << stRebRelBuff.rel_type << std::endl;
    for(VcRelationshipFeat_t::const_iterator itv = it->second.begin();
        itv != it->second.end(); ++itv)
    {
      if(stRebRelBuff.relationship_id != itv->relationship_id)
      {
        std::cerr << "WARNING:[rebuildRelationshipData-01] illegal relationship_id="
                  << itv->relationship_id << std::endl;
        return false;
      }
      else if(stRebRelBuff.rel_type != itv->rel_type)
      {
        std::cerr << "WARNING:[rebuildRelationshipData-02] illegal rel_type="
                  << itv->rel_type << std::endl;
        return false;
      }
      StDmpRebRelationshipFeat_t stRebRelFeatBuff;
      stRebRelFeatBuff.role_number = itv->role_number;
      //std::cerr << "DEBUG:[rebuildRelationshipData] role_number="
      //        << stRebRelFeatBuff.role_number << std::endl;

      stRebRelFeatBuff.feat_category_num = itv->feat_category_num;
      stRebRelFeatBuff.feature_id = itv->feature_id;
      stRebRelBuff.mVcDmpRebRelationshipFeat.push_back(stRebRelFeatBuff);
    }

    if(stRebRelBuff.rel_type == R2110_Connectivity)
      g_VcDmpRebRelationshipConnectivity.push_back(stRebRelBuff);  // [R2110]経路接続
    else if(stRebRelBuff.rel_type == R9110_Crossing)
      g_VcDmpRebRelationshipCrossing.push_back(stRebRelBuff);   // [R9110]交差
    else if(stRebRelBuff.rel_type == R9120_Adjacency)
      g_VcDmpRebRelationshipAdjacency.push_back(stRebRelBuff);  // [R9120]隣接
    else if(stRebRelBuff.rel_type == R9130_Branch)
      g_VcDmpRebRelationshipBranch.push_back(stRebRelBuff);     // [R9130]分岐
    else if(stRebRelBuff.rel_type == R9210_TrafficLightRegulationForLane)
      g_VcDmpRebRelationshipTrafficLightRegulationForLane.push_back(stRebRelBuff);  // [R9210]信号規制
    else if(stRebRelBuff.rel_type == R9220_TrafficSignRegulationForLane)
      g_VcDmpRebRelationshipTrafficSignRegulationForLane.push_back(stRebRelBuff);  // [R9220]標識規制
    else if(stRebRelBuff.rel_type == R9230_FeatureAlongWithLane)
      g_VcDmpRebRelationshipFeatureAlongWithLane.push_back(stRebRelBuff);  // [R9230]地物関連
    else
    {
      std::cerr << "WARNING:[rebuildRelationshipData-03] illegal rel_type="
                << stRebRelBuff.rel_type << ", relationship_id="
                << stRebRelBuff.relationship_id << std::endl;
      return false;
    }
  }

  return true;
}

/************************************************************
*  読み込みデータの確認表示
*
*    input: nothing
*    output: nothing
*    return: nothing
************************************************************/
void checkDmpData(void)
{
  if(!bDebugFlg)
    return;

  // データレコード数
  ofsdbg << std::endl;
  ofsdbg << "[input static dynamic-map data count]" << std::endl;
  ofsdbg << "  attribute_composition = " << mMpVcAttributeComposition.size() << std::endl;
  ofsdbg << "  attribute_group       = " << mVcAttributeGroup.size() << std::endl;
  ofsdbg << "  cmp_feat_part_attr    = " << mVcMpVcCmpFeatPartAttr[0].size()
                                               + mVcMpVcCmpFeatPartAttr[1].size()
                                               + mVcMpVcCmpFeatPartAttr[2].size()
                                               + mVcMpVcCmpFeatPartAttr[3].size() << std::endl;
  ofsdbg << "  feature_attr          = " << mVcMpFeatureAttr[0].size()
                                               + mVcMpFeatureAttr[1].size()
                                               + mVcMpFeatureAttr[2].size()
                                               + mVcMpFeatureAttr[3].size() << std::endl;
  ofsdbg << "  feature_category      = " << mMpFeatureCategory.size() << std::endl;
  ofsdbg << "  feature_class_code    = " << mMpFeatureClassCode.size() << std::endl;
  ofsdbg << "  pf_area_feature       = " << mMpPfAreaFeature.size() << std::endl;
  ofsdbg << "  pf_area_topo_prim     = " << mMpPfAreaTopoPrim.size() << std::endl;
  ofsdbg << "  pf_comp_feat_part     = " << mMpVcPfCompFeatPart.size() << std::endl;
  ofsdbg << "  pf_comp_feature       = " << mMpPfCompFeature.size() << std::endl;
  ofsdbg << "  pf_line_feature       = " << mMpPfLineFeature.size() << std::endl;
  ofsdbg << "  pf_line_topo_prim     = " << mMpVcPfLineTopoPrim.size() << std::endl;
  ofsdbg << "  pf_point_feature      = " << mMpPfPointFeature.size() << std::endl;
  ofsdbg << "  relationship          = " << mMpRelationship.size() << std::endl;
  ofsdbg << "  relationship_attr     = " << mMpVcRelationshipAttr.size() << std::endl;
  ofsdbg << "  relationship_feat     = " << mMpVcRelationshipFeat.size() << std::endl;
  ofsdbg << "  st_edge               = " << mMpStEdge.size() << std::endl;
  ofsdbg << "  st_face               = " << mMpStFace.size() << std::endl;
  ofsdbg << "  st_node               = " << mMpStNode.size() << std::endl << std::endl;

  // 再構築した地物データ数
  ofsdbg << "[input static dynamic-map feature count]" << std::endl;
  ofsdbg << "  FP4120_Junction              = " << g_MpDmpJunction.size() << std::endl;
  ofsdbg << "  FC8110_Lane                  = " << g_MpDmpLane.size() << std::endl;
  ofsdbg << "  FC8140_ExtendedPedestrian    = " << g_MpDmpExtendedPedestrian.size() << std::endl;
  ofsdbg << "  FC8145_ExtendedPedestrianCrossing = " << g_MpDmpExtendedPedestrianCrossing.size() << std::endl;
  ofsdbg << "  FA8170_IntersectionAreaShape = " << g_MpDmpIntersectionAreaShape.size() << std::endl;
  ofsdbg << "  FP8210_Pole                  = " << g_MpDmpPole.size() << std::endl;
  ofsdbg << "  FC8220_ExtendedTrafficSign   = " << g_MpDmpExtendedTrafficSign.size() << std::endl;
  ofsdbg << "  FC8230_ExtendedTrafficLight  = " << g_MpDmpExtendedTrafficLight.size() << std::endl;
  ofsdbg << "  FC8240_ExtendedLighting      = " << g_MpDmpExtendedLighting.size() << std::endl;
  ofsdbg << "  FC8250_ExtendedRoadMarkings  = " << g_MpDmpExtendedRoadMarkings.size() << std::endl;
  ofsdbg << "  FL8260_StopLine              = " << g_MpDmpStopLine.size() << std::endl;
  ofsdbg << "  FL8310_RoadEdge              = " << g_MpDmpRoadEdge.size() << std::endl;
  ofsdbg << "  FL8311_Curb                  = " << g_MpDmpCurb.size() << std::endl;
  ofsdbg << "  FA8312_Gutter                = " << g_MpDmpGutter.size() << std::endl;
  ofsdbg << "  FA8313_GuardRail             = " << g_MpDmpGuardRail.size() << std::endl;
  ofsdbg << "  FA8314_ZebraZone             = " << g_MpDmpZebraZone.size() << std::endl << std::endl;

  // 再構築した関連データ数
  ofsdbg << "[input static dynamic-map relationship count]" << std::endl;
  ofsdbg << "  R2110_Connectivity = " << g_VcDmpRebRelationshipConnectivity.size() << std::endl;
  ofsdbg << "  R9110_Crossing     = " << g_VcDmpRebRelationshipCrossing.size() << std::endl;
  ofsdbg << "  R9120_Adjacency    = " << g_VcDmpRebRelationshipAdjacency.size() << std::endl;
  ofsdbg << "  R9130_Branch       = " << g_VcDmpRebRelationshipBranch.size() << std::endl;
  ofsdbg << "  R9210_TrafficLightRegulationForLane = " << g_VcDmpRebRelationshipTrafficLightRegulationForLane.size() << std::endl;
  ofsdbg << "  R9220_TrafficSignRegulationForLane  = " << g_VcDmpRebRelationshipTrafficSignRegulationForLane.size() << std::endl;
  ofsdbg << "  R9230_FeatureAlongWithLane          = " << g_VcDmpRebRelationshipFeatureAlongWithLane.size() << std::endl << std::endl;

  // 属性データの検出数
  int nA_NONE=0, nA_CT=0, nA_DA=0, nA_DH=0, nA_DM=0, nA_DT=0, nA_DW=0, nA_DX=0,
      nA_DZ=0, nA_EV=0, nA_GR=0, nA_GU=0, nA_HT=0, nA_LD=0, nA_LI=0, nA_LM=0,
      nA_LR=0, nA_LY=0, nA_NL=0, nA_OY=0, nA_PN=0, nA_SP=0, nA_SY=0, nA_TR=0,
      nA_TS=0, nA_TT=0, nA_VT=0, nA_WI=0, nA_WL=0, nA_WR=0, nA_XM=0, nA_YN=0;
  for(MpVcAttributeComposition_t::const_iterator it = mMpVcAttributeComposition.begin();
      it != mMpVcAttributeComposition.end(); ++it)
  {
    for(std::vector<StAttributeComposition_t>::const_iterator itc = it->second.begin();
        itc != it->second.end(); ++itc)
    {
      if(itc->av_type_code == A_NONE)    nA_NONE++;    
      else if(itc->av_type_code == A_CT) nA_CT++;
      else if(itc->av_type_code == A_DA) nA_DA++;
      else if(itc->av_type_code == A_DH) nA_DH++;
      else if(itc->av_type_code == A_DM) nA_DM++;
      else if(itc->av_type_code == A_DT) nA_DT++;
      else if(itc->av_type_code == A_DW) nA_DW++;
      else if(itc->av_type_code == A_DX) nA_DX++;
      else if(itc->av_type_code == A_DZ) nA_DZ++;
      else if(itc->av_type_code == A_EV) nA_EV++;
      else if(itc->av_type_code == A_GR) nA_GR++;
      else if(itc->av_type_code == A_GU) nA_GU++;
      else if(itc->av_type_code == A_HT) nA_HT++;
      else if(itc->av_type_code == A_LD) nA_LD++;
      else if(itc->av_type_code == A_LI) nA_LI++;
      else if(itc->av_type_code == A_LM) nA_LM++;
      else if(itc->av_type_code == A_LR) nA_LR++;
      else if(itc->av_type_code == A_LY) nA_LY++;
      else if(itc->av_type_code == A_NL) nA_NL++;
      else if(itc->av_type_code == A_OY) nA_OY++;
      else if(itc->av_type_code == A_PN) nA_PN++;
      else if(itc->av_type_code == A_SP) nA_SP++;
      else if(itc->av_type_code == A_SY) nA_SY++;
      else if(itc->av_type_code == A_TR) nA_TR++;
      else if(itc->av_type_code == A_TS) nA_TS++;
      else if(itc->av_type_code == A_TT) nA_TT++;
      else if(itc->av_type_code == A_VT) nA_VT++;
      else if(itc->av_type_code == A_WI) nA_WI++;
      else if(itc->av_type_code == A_WL) nA_WL++;
      else if(itc->av_type_code == A_WR) nA_WR++;
      else if(itc->av_type_code == A_XM) nA_XM++;
      else if(itc->av_type_code == A_YN) nA_YN++;
      else
        std::cerr << "WARNING:[checkDmpData-01] illegal av_type_code="
                  << itc->av_type_code << std::endl;
    }
  }

  ofsdbg << "[input static dynamic-map attibute count]" << std::endl;
  ofsdbg << "  NONE = " << nA_NONE << std::endl;
  ofsdbg << "  CT   = " << nA_CT << std::endl;
  ofsdbg << "  DA   = " << nA_DA << std::endl;
  ofsdbg << "  DH   = " << nA_DH << std::endl;
  ofsdbg << "  DM   = " << nA_DM << std::endl;
  ofsdbg << "  DT   = " << nA_DT << std::endl;
  ofsdbg << "  DW   = " << nA_DW << std::endl;
  ofsdbg << "  DX   = " << nA_DX << std::endl;
  ofsdbg << "  DZ   = " << nA_DZ << std::endl;
  ofsdbg << "  EV   = " << nA_EV << std::endl;
  ofsdbg << "  GR   = " << nA_GR << std::endl;
  ofsdbg << "  GU   = " << nA_GU << std::endl;
  ofsdbg << "  HT   = " << nA_HT << std::endl;
  ofsdbg << "  LD   = " << nA_LD << std::endl;
  ofsdbg << "  LI   = " << nA_LI << std::endl;
  ofsdbg << "  LM   = " << nA_LM << std::endl;
  ofsdbg << "  LR   = " << nA_LR << std::endl;
  ofsdbg << "  LY   = " << nA_LY << std::endl;
  ofsdbg << "  NL   = " << nA_NL << std::endl;
  ofsdbg << "  OY   = " << nA_OY << std::endl;
  ofsdbg << "  PN   = " << nA_PN << std::endl;
  ofsdbg << "  SP   = " << nA_SP << std::endl;
  ofsdbg << "  SY   = " << nA_SY << std::endl;
  ofsdbg << "  TR   = " << nA_TR << std::endl;
  ofsdbg << "  TS   = " << nA_TS << std::endl;
  ofsdbg << "  TT   = " << nA_TT << std::endl;
  ofsdbg << "  VT   = " << nA_VT << std::endl;
  ofsdbg << "  WI   = " << nA_WI << std::endl;
  ofsdbg << "  WL   = " << nA_WL << std::endl;
  ofsdbg << "  WR   = " << nA_WR << std::endl;
  ofsdbg << "  XM   = " << nA_XM << std::endl;
  ofsdbg << "  YN   = " << nA_YN << std::endl << std::endl;

  return;
}

/************************************************************
*  地物クラスコード文字列の数値化
*
*    input: ch - 地物クラスコード文字列 (4文字)
*    output: nothing
*    return: 地物クラスコード
************************************************************/
static DmpFtrClass_e getFeatureClassCode(const char *ch)
{
  std::string str(ch);

  if(str == "4110")
    return FL4110_RoadElement;
  else if(str == "4115")
    return FL4115_Pathway;
  else if(str == "4120")
    return FP4120_Junction;
  else if(str == "4140")
    return FC4140_Road;
  else if(str == "4145")
    return FC4145_Intersection;
  else if(str == "7210")
    return FP7210_Signpost;
  else if(str == "7220")
    return FP7220_TrafficSign;
  else if(str == "7230")
    return FP7230_TrafficLight;
  else if(str == "7240")
    return FP7240_PedestrianCrossing;
  else if(str == "7245")
    return FC7245_ComplexPedestrianCrossing;
  else if(str == "7251")
    return FP7251_EnvironmentalEquipment;
  else if(str == "7252")
    return FP7252_Lighting;
  else if(str == "7254")
    return FP7254_RoadMarkings;
  else if(str == "8110")
    return FC8110_Lane;
  else if(str == "8120")
    return FA8120_LaneArea;
  else if(str == "8130")
    return FL8130_LaneLine;
  else if(str == "8140")
    return FC8140_ExtendedPedestrian;
  else if(str == "8145")
    return FC8145_ExtendedPedestrianCrossing;
  else if(str == "8150")
    return FA8150_PedestrianArea;
  else if(str == "8160")
    return FL8160_PedestrianLine;
  else if(str == "8170")
    return FA8170_IntersectionAreaShape;
  else if(str == "8210")
    return FP8210_Pole;
  else if(str == "8220")
    return FC8220_ExtendedTrafficSign;
  else if(str == "8230")
    return FC8230_ExtendedTrafficLight;
  else if(str == "8231")
    return FP8231_TrafficLightLamp;
  else if(str == "8240")
    return FC8240_ExtendedLighting;
  else if(str == "8241")
    return FL8241_LightingLamp;
  else if(str == "8250")
    return FC8250_ExtendedRoadMarkings;
  else if(str == "8251")
    return FA8251_RoadMarkingsShape;
  else if(str == "8252")
    return FA8252_PedestrianCrossingMarkingShape;
  else if(str == "8260")
    return FL8260_StopLine;
  else if(str == "8310")
    return FL8310_RoadEdge;
  else if(str == "8311")
    return FL8311_Curb;
  else if(str == "8312")
    return FA8312_Gutter;
  else if(str == "8313")
    return FA8313_GuardRail;
  else if(str == "8314")
    return FA8314_ZebraZone;
  else if(str == "8410")
    return FP8410_ShapeDescriptionPoint;

  std::cerr << "WARNING:[getFeatureClassCode-01] illegal str=[" << str << "]" << std::endl;
  return F_NONE;
}

/************************************************************
*  関連タイプ文字列の数値化
*
*    input: ch - 関連タイプ文字列 (4文字)
*    output: nothing
*    return: 関連タイプ
************************************************************/
static DmpRelType_e getRelationshipType(const char *ch)
{
  std::string str(ch);

  if(str == "2110")
    return R2110_Connectivity;
  else if(str == "9110")
    return R9110_Crossing;
  else if(str == "9120")
    return R9120_Adjacency;
  else if(str == "9130")
    return R9130_Branch;
  else if(str == "9210")
    return R9210_TrafficLightRegulationForLane;
  else if(str == "9220")
    return R9220_TrafficSignRegulationForLane;
  else if(str == "9230")
    return R9230_FeatureAlongWithLane;

  std::cerr << "WARNING:[getRelationshipType-01] illegal str=[" << str << "]" << std::endl;
  return R_NONE;
}

/************************************************************
*  属性コード文字列の数値化
*
*    input: ch - 属性コード文字列 (2文字)
*    output: nothing
*    return: 属性コード
************************************************************/
static DmpAtrCode_e getAttributeCode(const char *ch)
{
  std::string str(ch);

  if(str == "CT")
    return A_CT;
  else if(str == "DA")
    return A_DA;
  else if(str == "DH")
    return A_DH;
  else if(str == "DM")
    return A_DM;
  else if(str == "DT")
    return A_DT;
  else if(str == "DW")
    return A_DW;
  else if(str == "DX")
    return A_DX;
  else if(str == "DZ")
    return A_DZ;
  else if(str == "EV")
    return A_EV;
  else if(str == "GR")
    return A_GR;
  else if(str == "GU")
    return A_GU;
  else if(str == "HT")
    return A_HT;
  else if(str == "LD")
    return A_LD;
  else if(str == "LI")
    return A_LI;
  else if(str == "LM")
    return A_LM;
  else if(str == "LR")
    return A_LR;
  else if(str == "LY")
    return A_LY;
  else if(str == "NL")
    return A_NL;
  else if(str == "OY")
    return A_OY;
  else if(str == "PN")
    return A_PN;
  else if(str == "SP")
    return A_SP;
  else if(str == "SY")
    return A_SY;
  else if(str == "TR")
    return A_TR;
  else if(str == "TS")
    return A_TS;
  else if(str == "TT")
    return A_TT;
  else if(str == "VT")
    return A_VT;
  else if(str == "WI")
    return A_WI;
  else if(str == "WL")
    return A_WL;
  else if(str == "WR")
    return A_WR;
  else if(str == "XM")
    return A_XM;
  else if(str == "YN")
    return A_YN;

  std::cerr << "WARNING:[getAttributeCode-01] illegal str=[" << str << "]" << std::endl;
  return A_NONE;
}

/************************************************************
*  点座標情報文字列の数値化
*
*    input: str - 点座標情報文字列 (Point Z)
*           sno - 平面直角座標系の系番号(1-19)
*    output: pbuff - 点座標
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool getGeometryPointZ(std::string str, int sno, StDmpPoint_t& pbuff)
{
  // フォーマットのチェック
  if((str.size() < 15) ||
     (str.substr(0, 9) != "POINT Z (") ||
     (str.substr(str.size() - 1, 1) != ")"))
  {
    std::cerr << "WARNING:[getGeometryPointZ-01] illegal format, str=[" << str << "]" << std::endl;
    return false;    
  }

  // XYZ文字列を抽出してトークンに分解
  //std::cerr << "debug:[getGeometryPointZ] sxyz=[" << str.substr(9, str.size() - 10) << "]" << std::endl;
  std::istringstream stream(str.substr(9, str.size() - 10));
  std::string token;

  if(!getline(stream, token, ' '))
  {
    std::cerr << "WARNING:[getGeometryPointZ-02] illegal X, str=[" << str << "]" << std::endl;
    return false;    
  }
  pbuff.x = atof(token.c_str());

  if(!getline(stream, token, ' '))
  {
    std::cerr << "WARNING:[getGeometryPointZ-03] illegal Y str=[" << str << "]" << std::endl;
    return false;    
  }
  pbuff.y = atof(token.c_str());

  if(!getline(stream, token))
  {
    std::cerr << "WARNING:[getGeometryPointZ-04] illegal Z, str=[" << str << "]" << std::endl;
    return false;    
  }
  pbuff.z = atof(token.c_str());

  pbuff.sno = sno;

  calXyToBl(pbuff.x, pbuff.y, sno, pbuff.yLat, pbuff.xLon);
  pbuff.xLon *= 180.0 / M_PI;
  pbuff.yLat *= 180.0 / M_PI;

  return true;
}

/************************************************************
*  線座標情報文字列の数値化
*
*    input: str - 線座標情報文字列 (Linestring Z)
*           sno - 平面直角座標系の系番号(1-19)
*    output: lbuff - 線座標 (点座標配列)
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool getGeometryLinestringZ(std::string str, int sno, VcDmpPoint_t& lbuff)
{
  // フォーマットのチェック
  if((str.size() < 15) ||
     (str.substr(0, 14) != "LINESTRING Z (") ||
     (str.substr(str.size() - 1, 1) != ")"))
  {
    std::cerr << "WARNING:[getGeometryLinestringZ-01] illegal format, str=[" << str << "]" << std::endl;
    return false;    
  }

  // XYZ文字列を抽出してトークンに分解
  //std::cerr << "debug:[getGeometryLinestringZ] sxyz=[" << str.substr(14, str.size() - 15) << "]" << std::endl;
  std::istringstream stream(str.substr(14, str.size() - 15));
  std::string token;
  StDmpPoint_t pbuff;
  lbuff.clear();

  for(int i=0; ; i++)
  {
    if(!getline(stream, token, ' '))
    {
      if(i > 0)
        return true;

      std::cerr << "WARNING:[getGeometryLinestringZ-02] illegal X, str=[" << str << "]" << std::endl;
      return false;    
    }
    pbuff.x = atof(token.c_str());

    if(!getline(stream, token, ' '))
    {
      std::cerr << "WARNING:[getGeometryLinestringZ-03] illegal Y str=[" << str << "]" << std::endl;
      return false;    
    }
    pbuff.y = atof(token.c_str());

    if(!getline(stream, token, ','))
    {
      std::cerr << "WARNING:[getGeometryLinestringZ-04] illegal Z, str=[" << str << "]" << std::endl;
      return false;    
    }
    pbuff.z = atof(token.c_str());

    pbuff.sno = sno;

    calXyToBl(pbuff.x, pbuff.y, sno, pbuff.yLat, pbuff.xLon);
    pbuff.xLon *= 180.0 / M_PI;
    pbuff.yLat *= 180.0 / M_PI;

    lbuff.push_back(pbuff);
  }
}

/************************************************************
*  面座標情報文字列の数値化
*
*    input: str - 面座標情報文字列 (Polygon Z)
*           sno - 平面直角座標系の系番号(1-19)
*    output: abuff - 面座標 (点座標配列)
*    return: リターンコード (true=成功、false=失敗)
*    note: 始点と終点の座標が同じ
************************************************************/
static bool getGeometryPolygonZ(std::string str, int sno, VcDmpPoint_t& abuff)
{
  // フォーマットのチェック
  if((str.size() < 12) ||
     (str.substr(0, 12) != "POLYGON Z ((") ||
     (str.substr(str.size() - 2, 2) != "))"))
  {
    std::cerr << "WARNING:[getGeometryPolygonZ-01] illegal format, str=[" << str << "]" << std::endl;
    return false;    
  }

  // XYZ文字列を抽出してトークンに分解
  //std::cerr << "debug:[getGeometryPolygonZ] sxyz=[" << str.substr(12, str.size() - 14) << "]" << std::endl;
  std::istringstream stream(str.substr(12, str.size() - 14));
  std::string token;
  StDmpPoint_t pbuff;
  abuff.clear();

  for(int i=0; ; i++)
  {
    if(!getline(stream, token, ' '))
    {
      if(i > 0)
        return true;

      std::cerr << "WARNING:[getGeometryPolygonZ-02] illegal X, str=[" << str << "]" << std::endl;
      return false;    
    }
    pbuff.x = atof(token.c_str());

    if(!getline(stream, token, ' '))
    {
      std::cerr << "WARNING:[getGeometryPolygonZ-03] illegal Y str=[" << str << "]" << std::endl;
      return false;    
    }
    pbuff.y = atof(token.c_str());

    if(!getline(stream, token, ','))
    {
      std::cerr << "WARNING:[getGeometryPolygonZ-04] illegal Z, str=[" << str << "]" << std::endl;
      return false;    
    }
    pbuff.z = atof(token.c_str());

    pbuff.sno = sno;

    calXyToBl(pbuff.x, pbuff.y, sno, pbuff.yLat, pbuff.xLon);
    pbuff.xLon *= 180.0 / M_PI;
    pbuff.yLat *= 180.0 / M_PI;

    abuff.push_back(pbuff);
  }
}

#if 0
/************************************************************
*  地物の関連情報を抽出する
*
*    input: fcat - 地物カテゴリ
*           fid  - 地物ID (キー)
*    output: nothing
*    return: 関連値(relationship_feat + relationship)の配列 (NULL=失敗)
************************************************************/
static VcRelationshipFeat_t *getFeatureRelationship(DmpFtrCat_e fcat, int fid)
{
  if((fcat < CAT_Point) || (fcat > CAT_Complex))
  {
    std::cerr << "WARNING:[getFeatureAttribute-01] illegal fcat=" << fcat << std::endl;
    return NULL;
  }

  // feature_attr の検索
  std::map<int, StFeatureAttr_t>::const_iterator
    itfa = mVcMpFeatureAttr[fcat - 1].find(fid);
  if(itfa == mVcMpFeatureAttr[fcat - 1].end())
  {
    std::cerr << "WARNING:[getFeatureAttribute-02] illegal feature_id="
              << fid << std::endl;
    return NULL;
  }

  return NULL;
}
#endif

/************************************************************
*  地物の属性情報を抽出する
*
*    input: fcat - 地物カテゴリ
*           fid  - 地物ID (キー)
*    output: nothing
*    return: 属性値(attribute_composition)の配列 (NULL=失敗)
************************************************************/
static VcAttributeComposition_t *getFeatureAttribute(DmpFtrCat_e fcat, int fid)
{
  if((fcat < CAT_Point) || (fcat > CAT_Complex))
  {
    std::cerr << "WARNING:[getFeatureAttribute-01] illegal fcat=" << fcat << std::endl;
    return NULL;
  }

  // feature_attr の検索
  std::map<int, StFeatureAttr_t>::const_iterator
    itfa = mVcMpFeatureAttr[fcat - 1].find(fid);
  if(itfa == mVcMpFeatureAttr[fcat - 1].end())
  {
    std::cerr << "WARNING:[getFeatureAttribute-02] illegal feature_id="
              << fid << std::endl;
    return NULL;
  }

  // attribute_composition の検索
  MpVcAttributeComposition_t::const_iterator
    itac = mMpVcAttributeComposition.find(itfa->second.attribute_id);
  if(itac == mMpVcAttributeComposition.end())
  {
    std::cerr << "WARNING:[getFeatureAttribute-03] illegal attribute_id="
              << itfa->second.attribute_id << std::endl;
    return NULL;
  }

  return (VcAttributeComposition_t *)&(itac->second);
}

/************************************************************
*  平面直角座標を換算して経緯度を求める
*    座標系のX軸は北方向を正とし、Y軸は東方向を正とする。
*
*    input: x - 平面直角座標系 X座標(緯度方向) [m]
*           y - 平面直角座標系 Y座標(経度方向) [m]
*           sno - 系番号(1-19)
*    output: phi - 緯度 [rad]
*            lmd - 経度 [rad]
*    return: nothing
*
*    note: 平面直角座標原点は、神奈川県(=9系) (緯度:139度50分0秒、経度:36度0分0秒)
*    ref: http://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/algorithm/xy2bl/xy2bl.htm
************************************************************/
static void calXyToBl(double x, double y, int sno, double& phi, double& lmd)
{
  // 座標原点の緯度経度
  double phi0 = 0; // 原点緯度 [rad]
  double lmd0 = 0; // 原点経度 [rad]
  switch(sno)
  {
    case 1:
      // 1系座標原点の緯度経度(129度30分0秒、33度0分0秒)
      phi0 = (33) * M_PI / 180.0;
      lmd0 = (129 + 30/60.0) * M_PI / 180.0;
      break;

    case 2:
      // 2系座標原点の緯度経度(131度0分0秒、33度0分0秒)
      phi0 = (33) * M_PI / 180.0;
      lmd0 = (131 + 0/60.0) * M_PI / 180.0;
      break;

    case 3:
      // 3系座標原点の緯度経度(132度10分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (132 + 10/60.0) * M_PI / 180.0;
      break;

    case 4:
      // 4系座標原点の緯度経度(133度30分0秒、33度0分0秒)
      phi0 = (33) * M_PI / 180.0;
      lmd0 = (133 + 30/60.0) * M_PI / 180.0;
      break;

    case 5:
      // 5系座標原点の緯度経度(134度20分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (134 + 20/60.0) * M_PI / 180.0;
      break;

    case 6:
      // 6系座標原点の緯度経度(136度0分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (136 + 0/60.0) * M_PI / 180.0;
      break;

    case 7:
      // 7系座標原点の緯度経度(137度10分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (137 + 10/60.0) * M_PI / 180.0;
      break;

    case 8:
      // 8系座標原点の緯度経度(138度30分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (138 + 30/60.0) * M_PI / 180.0;
      break;

    case 9:
      // 9系座標原点の緯度経度(139度50分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (139 + 50/60.0) * M_PI / 180.0;
      break;

    case 10:
      // 10系座標原点の緯度経度(140度50分0秒、40度0分0秒)
      phi0 = (40) * M_PI / 180.0;
      lmd0 = (140 + 0/60.0) * M_PI / 180.0;
      break;

    case 11:
      // 11系座標原点の緯度経度(140度15分0秒、44度0分0秒)
      phi0 = (44) * M_PI / 180.0;
      lmd0 = (135 + 0/60.0) * M_PI / 180.0;
      break;

    case 12:
      // 12系座標原点の緯度経度(142度15分0秒、44度0分0秒)
      phi0 = (44) * M_PI / 180.0;
      lmd0 = (142 + 15/60.0) * M_PI / 180.0;
      break;

    case 13:
      // 13系座標原点の緯度経度(144度15分0秒、44度0分0秒)
      phi0 = (44) * M_PI / 180.0;
      lmd0 = (144 + 15/60.0) * M_PI / 180.0;
      break;

    case 14:
      // 14系座標原点の緯度経度(142度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (142 + 0/60.0) * M_PI / 180.0;
      break;

    case 15:
      // 15系座標原点の緯度経度(127度30分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (127 + 30/60.0) * M_PI / 180.0;
      break;

    case 16:
      // 16系座標原点の緯度経度(124度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (124 + 0/60.0) * M_PI / 180.0;
      break;

    case 17:
      // 17系座標原点の緯度経度(131度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (131 + 0/60.0) * M_PI / 180.0;
      break;

    case 18:
      // 18系座標原点の緯度経度(136度 0分0秒、20度0分0秒)
      phi0 = (20) * M_PI / 180.0;
      lmd0 = (136 + 0/60.0) * M_PI / 180.0;
      break;

    case 19:
      // 19系座標原点の緯度経度(154度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (154 + 0/60.0) * M_PI / 180.0;
      break;

    default:
      std::cerr << "ERROR:[calXyToBl-01] illegal sno=" << sno << std::endl;
      return;
  }

  // サンプルターゲットの緯度経度
  //  緯度:  36.50.25.0000
  //  経度: 138.35.45.2500
  //double phi = (36 + 50/60.0 + 25.0/3600.0) * M_PI / 180.0; // 緯度 [rad]
  //double lmd = (138 + 35/60.0 + 45.25/3600.0) * M_PI / 180.0; // 経度 [rad]

  // パラメータ（世界測地系-測地基準系1980(GRS80)楕円体）
  double a = 6378137.0;      // 楕円体の長(赤道)半径[ｍ]
  double F = 298.257222101;  // 楕円体の逆扁平率(1/f)
  double m0 = 0.9999;        // 平面直角座標系のX軸上における縮尺係数

  double n  = 1/(2*F - 1);
  double n2 = n*n;
  double n3 = n2*n;
  double n4 = n3*n;
  double n5 = n4*n;
  double n6 = n5*n;

  double b1 =      n/2
             -   2*n2/3
             +  37*n3/96
             -   1*n4/360
             -  81*n5/512;

  double b2 =      n2/48
             +     n3/15
             - 437*n4/1440
             +  46*n5/105;

  double b3 =   17*n3/480
             -  37*n4/840
             - 209*n5/4480;

  double b4 = 4397*n4/161280
              - 11*n5/504;

  double b5 = 4583*n5/161280;

  double d1 =     2*n
             -    2*n2/3
             -    2*n3
             +  116*n4/45
             +   26*n5/45
             - 2854*n6/675;

  double d2 =     7*n2/3
             -    8*n3/5
             -  227*n4/45
             + 2704*n5/315
             + 2323*n6/945;

  double d3 =     56*n3/15
             -   136*n4/35
             -  1262*n5/105
             + 73814*n6/2835;

  double d4 =    4279*n4/630
             -    332*n5/35
             - 399572*n6/14175;

  double d5 =    4174*n5/315
             - 144838*n6/6237;

  double d6 =  601676*n6/22275;

  double A0 = 1 + n2/4 + n4/64;

  double A1 = -3*(n - n3/8 - n5/64)/2;

  double A2 = 15*(n2 - n4/4)/16;

  double A3 = -35*(n3 - 5*n5/16)/48;

  double A4 = 315*n4/512;

  double A5 = -693*n5/1280;

  double sbar = (m0 * a)/(1 + n)*(A0*phi0
                                + A1*sin(2*phi0)
                                + A2*sin(4*phi0)
                                + A3*sin(6*phi0)
                                + A4*sin(8*phi0)
                                + A5*sin(10*phi0));

  double abar = (m0 * a)/(1 + n)*A0;

  double xi = (x + sbar)/abar;  // ξ

  double et = y/abar;  // η

  double xi1 = xi  // ξ'
              - b1*sin(2*xi)*cosh(2*et)
              - b2*sin(4*xi)*cosh(4*et)
              - b3*sin(6*xi)*cosh(6*et)
              - b4*sin(8*xi)*cosh(8*et)
              - b5*sin(10*xi)*cosh(10*et);

  double et1 = et  // η'
              - b1*cos(2*xi)*sinh(2*et)
              - b1*cos(4*xi)*sinh(4*et)
              - b1*cos(6*xi)*sinh(6*et)
              - b1*cos(8*xi)*sinh(8*et)
              - b1*cos(10*xi)*sinh(10*et);

/*
  double sg1 = 1  // σ'
              - 2*b1*cos(2*xi)*cosh(2*et)
              - 4*b1*cos(4*xi)*cosh(4*et)
              - 6*b1*cos(6*xi)*cosh(6*et)
              - 8*b1*cos(8*xi)*cosh(8*et)
              - 10*b1*cos(10*xi)*cosh(10*et);

  double tu1 =  2*b1*sin(2*xi)*sinh(2*et)  // τ'
              + 4*b1*sin(4*xi)*sinh(4*et)
              + 6*b1*sin(6*xi)*sinh(6*et)
              + 8*b1*sin(8*xi)*sinh(8*et)
              + 10*b1*sin(10*xi)*sinh(10*et);
*/

  double ch = asin(sin(xi1)/cosh(et1));  // χ

  phi = ch  // φ
       + d1*sin(2*ch)
       + d2*sin(4*ch)
       + d3*sin(6*ch)
       + d4*sin(8*ch)
       + d5*sin(10*ch)
       + d6*sin(12*ch);

  lmd = lmd0 + atan(sinh(et1)/cos(xi1));  // λ

  return;
  //x=   93948.2818 [m]
  //y= -110367.1186 [m]
}

#if 0
/************************************************************
*  経緯度を換算して平面直角座標を求める
*    座標系のX軸は北方向を正とし、Y軸は東方向を正とする。
*
*    input: phi - 緯度 [rad]
*           lmd - 経度 [rad]
*           sno - 系番号(1-19)
*    output: x - 平面直角座標系 X座標(緯度方向) [m]
*            y - 平面直角座標系 Y座標(経度方向) [m]
*    return: nothing
*
*    note: 平面直角座標原点は、神奈川県(=9系) (緯度:139度50分0秒、経度:36度0分0秒)
*    ref: http://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/algorithm/bl2xy/bl2xy.htm
*         系番号 : 座標系原点 東経 : 北緯 : 地域
*           1     129度30分0秒  33度0分0秒  長崎・鹿児島奄美地方
*           2     131度0分0秒   33度0分0秒  九州地方
*           3     132度10分0秒  36度0分0秒  中国西地方
*           4     133度30分0秒  33度0分0秒  四国地方
*           5     134度20分0秒  36度0分0秒  中国東地方
*           6     136度0分0秒   36度0分0秒  近畿地方
*           7     137度10分0秒  36度0分0秒  北陸西・中部西地方
*           8     138度30分0秒  36度0分0秒  北陸東・中部東地方
*           9     139度50分0秒  36度0分0秒  関東地方
*           10    140度50分0秒  40度0分0秒  東北地方北部
*           11    140度15分0秒  44度0分0秒  北海道 西部
*           12    142度15分0秒  44度0分0秒  北海道 中部
*           13    144度15分0秒  44度0分0秒  北海道 東部
*           14    142度 0分0秒  26度0分0秒  東京都 小笠原
*           15    127度30分0秒  26度0分0秒  沖縄県 沖縄本島
*           16    124度 0分0秒  26度0分0秒  沖縄県 石垣島
*           17    131度 0分0秒  26度0分0秒  沖縄県 南大東島
*           18    136度 0分0秒  20度0分0秒  東京都 沖ノ鳥島
*           19    154度 0分0秒  26度0分0秒  東京都 南鳥島
************************************************************/
static void calBlToXy(double phi, double lmd, int sno, double& x, double& y)
{
  // 座標原点の緯度経度
  double phi0 = 0; // 原点緯度 [rad]
  double lmd0 = 0; // 原点経度 [rad]
  switch(sno)
  {
    case 1:
      // 1系座標原点の緯度経度(129度30分0秒、33度0分0秒)
      phi0 = (33) * M_PI / 180.0;
      lmd0 = (129 + 30/60.0) * M_PI / 180.0;
      break;

    case 2:
      // 2系座標原点の緯度経度(131度0分0秒、33度0分0秒)
      phi0 = (33) * M_PI / 180.0;
      lmd0 = (131 + 0/60.0) * M_PI / 180.0;
      break;

    case 3:
      // 3系座標原点の緯度経度(132度10分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (132 + 10/60.0) * M_PI / 180.0;
      break;

    case 4:
      // 4系座標原点の緯度経度(133度30分0秒、33度0分0秒)
      phi0 = (33) * M_PI / 180.0;
      lmd0 = (133 + 30/60.0) * M_PI / 180.0;
      break;

    case 5:
      // 5系座標原点の緯度経度(134度20分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (134 + 20/60.0) * M_PI / 180.0;
      break;

    case 6:
      // 6系座標原点の緯度経度(136度0分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (136 + 0/60.0) * M_PI / 180.0;
      break;

    case 7:
      // 7系座標原点の緯度経度(137度10分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (137 + 10/60.0) * M_PI / 180.0;
      break;

    case 8:
      // 8系座標原点の緯度経度(138度30分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (138 + 30/60.0) * M_PI / 180.0;
      break;

    case 9:
      // 9系座標原点の緯度経度(139度50分0秒、36度0分0秒)
      phi0 = (36) * M_PI / 180.0;
      lmd0 = (139 + 50/60.0) * M_PI / 180.0;
      break;

    case 10:
      // 10系座標原点の緯度経度(140度50分0秒、40度0分0秒)
      phi0 = (40) * M_PI / 180.0;
      lmd0 = (140 + 0/60.0) * M_PI / 180.0;
      break;

    case 11:
      // 11系座標原点の緯度経度(140度15分0秒、44度0分0秒)
      phi0 = (44) * M_PI / 180.0;
      lmd0 = (135 + 0/60.0) * M_PI / 180.0;
      break;

    case 12:
      // 12系座標原点の緯度経度(142度15分0秒、44度0分0秒)
      phi0 = (44) * M_PI / 180.0;
      lmd0 = (142 + 15/60.0) * M_PI / 180.0;
      break;

    case 13:
      // 13系座標原点の緯度経度(144度15分0秒、44度0分0秒)
      phi0 = (44) * M_PI / 180.0;
      lmd0 = (144 + 15/60.0) * M_PI / 180.0;
      break;

    case 14:
      // 14系座標原点の緯度経度(142度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (142 + 0/60.0) * M_PI / 180.0;
      break;

    case 15:
      // 15系座標原点の緯度経度(127度30分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (127 + 30/60.0) * M_PI / 180.0;
      break;

    case 16:
      // 16系座標原点の緯度経度(124度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (124 + 0/60.0) * M_PI / 180.0;
      break;

    case 17:
      // 17系座標原点の緯度経度(131度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (131 + 0/60.0) * M_PI / 180.0;
      break;

    case 18:
      // 18系座標原点の緯度経度(136度 0分0秒、20度0分0秒)
      phi0 = (20) * M_PI / 180.0;
      lmd0 = (136 + 0/60.0) * M_PI / 180.0;
      break;

    case 19:
      // 19系座標原点の緯度経度(154度 0分0秒、26度0分0秒)
      phi0 = (26) * M_PI / 180.0;
      lmd0 = (154 + 0/60.0) * M_PI / 180.0;
      break;

    default:
      std::cerr << "ERROR:[calBlToXy-01] illegal sno=" << sno << std::endl;
      return;
  }

  // サンプルターゲットの緯度経度
  //  緯度:  36.50.25.0000
  //  経度: 138.35.45.2500
  //double phi = (36 + 50/60.0 + 25.0/3600.0) * M_PI / 180.0; // 緯度 [rad]
  //double lmd = (138 + 35/60.0 + 45.25/3600.0) * M_PI / 180.0; // 経度 [rad]

  // パラメータ（世界測地系-測地基準系1980(GRS80)楕円体）
  double a = 6378137.0;      // 楕円体の長(赤道)半径[ｍ]
  double F = 298.257222101;  // 楕円体の逆扁平率(1/f)
  double m0 = 0.9999;        // 平面直角座標系のX軸上における縮尺係数

  double n  = 1/(2*F - 1);
  double n2 = n*n;
  double n3 = n2*n;
  double n4 = n3*n;
  double n5 = n4*n;
  double t = sinh(atanh(sin(phi))
              - 2*sqrt(n)/(1+n)*atanh(fabs(2*sqrt(n)/(1+n)*sin(phi))));
  double tbar = sqrt(1 + t*t);
  double lmdc = cos(lmd - lmd0);
  double lmds = sin(lmd - lmd0);
  double xi  = atan(t / lmdc);
  double eta = atanh(lmds / tbar);

  double a1 =       n/2
             -   2*n2/3
             +   5*n3/16
             +  41*n4/180
             - 127*n5/288;

  double a2 =   13*n2/48
             -   3*n3/5
             + 557*n4/1440
             + 281*n5/630;

  double a3 =     61*n3/240
             -   103*n4/140
             + 15061*n5/26880;

  double a4 = 49561*n4/161280
              - 179*n5/168;

  double a5 = 34729*n5/80640;

  double A0 = 1 + n2/4 + n4/64;

  double A1 = -3*(n - n3/8 - n5/64)/2;

  double A2 = 15*(n2 - n4/4)/16;

  double A3 = -35*(n3 - 5*n5/16)/48;

  double A4 = 315*n4/512;

  double A5 = -693*n5/1280;

  double sbar = (m0 * a)/(1 + n)*( A0*phi0
                                 + A1*sin(2*phi0)
                                 + A2*sin(4*phi0)
                                 + A3*sin(6*phi0)
                                 + A4*sin(8*phi0)
                                 + A5*sin(10*phi0));

  double abar = (m0 * a)/(1 + n)*A0;

  x = abar*(xi + a1*sin(2*xi)*cosh(2*eta)
               + a2*sin(4*xi)*cosh(4*eta)
               + a3*sin(6*xi)*cosh(6*eta)
               + a4*sin(8*xi)*cosh(8*eta)
               + a5*sin(10*xi)*cosh(10*eta)) - sbar;

  y = abar*(eta + a1*cos(2*xi)*sinh(2*eta)
                + a2*cos(4*xi)*sinh(4*eta)
                + a3*cos(6*xi)*sinh(6*eta)
                + a4*cos(8*xi)*sinh(8*eta)
                + a5*cos(10*xi)*sinh(10*eta));

  //x=   93948.2818 [m]
  //y= -110367.1186 [m]
}
#endif
