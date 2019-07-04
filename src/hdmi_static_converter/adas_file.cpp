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
*  adas_file.cpp
*  ADASマップファイルの読み書き処理
****************************************/

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>

#include <stdlib.h>
#ifdef QT_CORE_LIB
 #define _QT
#endif

#ifdef _QT
  #include <QDir>
  #include <QFile>
#else
  #include <sys/stat.h>
  //#include <sys/types.h>
#endif

#include "adas_data.h"
#include "adas_file.h"
#include "commonlib.h"

/************************************************************
*  グローバル変数定義
************************************************************/

// インデックス
MpAdIndex_t g_MpAdIndex;

// ポイントデータ (B01)
MpAdPoint_t g_MpAdPoint;

// ベクトルデータ (B02)
MpAdVector_t g_MpAdVector;

// ポールデータ (B03)
MpAdPole_t g_MpAdPole;

// ラインデータ (B04)
MpAdLine_t g_MpAdLine;

// エリアデータ (B05)
MpAdArea_t g_MpAdArea;

// ボックスデータ (B06)
MpAdBox_t g_MpAdBox;

// 中心線形データ (N01)
MpAdDtlane_t g_MpAdDtlane;

// ノードデータ (N02)
MpAdNode_t g_MpAdNode;

// レーンデータ個別 (N03)
MpAdLane_t g_MpAdLane;

// 走行エリアデータ (N04)
MpAdWayarea_t g_MpAdWayarea;

// 道路縁 (R001)
MpAdRoadedge_t g_MpAdRoadedge;

// 側溝 (R002)
MpAdGutter_t g_MpAdGutter;

// 縁石 (R003)
MpAdCurb_t g_MpAdCurb;

// 交差点 (R004)
MpAdIntersection_t g_MpAdIntersection;

// 路肩コンクリート (R005)
MpAdSidestrip_t g_MpAdSidestrip;

// 白線 (P001)
MpAdWhiteline_t g_MpAdWhiteline;

// 停止線 (P002)
MpAdStopline_t g_MpAdStopline;

// ゼブラゾーン (P003)
MpAdZebrazone_t g_MpAdZebrazone;

// 横断歩道 (P004)
MpAdCrosswalk_t g_MpAdCrosswalk;

// 路面マーク (P005)
MpAdRoadmark_t g_MpAdRoadmark;

// ガードレール (S001)
MpAdGuardrail_t g_MpAdGuardrail;

// 歩道 (S002)
MpAdSidewalk_t g_MpAdSidewalk;

// 車両乗入部 (S003)
MpAdDriveon_t g_MpAdDriveon;

// ポール (K001)
MpAdPoledata_t g_MpAdPoledata;

// 電柱 (K002)
MpAdUtilitypole_t g_MpAdUtilitypole;

// 標識 (K003)
MpAdRoadsign_t g_MpAdRoadsign;

// 信号 (K004)
MpAdSignaldata_t g_MpAdSignaldata;

// 街灯 (K005)
MpAdStreetlight_t g_MpAdStreetlight;

// カーブミラー (K006)
MpAdCurvemirror_t g_MpAdCurvemirror;

// 壁面(K007)
MpAdWall_t g_MpAdWall;

// フェンス (K008)
MpAdFence_t g_MpAdFence;

// 踏切ゾーン(K009)
MpAdRailroad_t g_MpAdRailroad;

/************************************************************
*  ローカル変数定義
************************************************************/
namespace
{
  // 基本図形クラス／道路データファイルの対応
  const struct
  {
    std::string FNAME;    // ファイル名
    AdasKind_e  IDVALUE;  // 識別子内部値
  }
  ADAS_BASIC_ITEMS[] =
  {
    { "point.csv",   B01_Point },  // ポイントデータ (B01)
    { "vector.csv",  B02_Vector }, // ベクトルデータ (B02)
    { "pole.csv",    B03_Pole },   // ポールデータ (B03)
    { "line.csv",    B04_Line },   // ラインデータ (B04)
    { "area.csv",    B05_Area },   // エリアデータ (B05)
    { "box.csv",     B06_Box },    // ボックスデータ (B06)
    { "dtlane.csv",  N01_Dtlane }, // 中心線形データ (N01)
    { "node.csv",    N02_Node },   // ノードデータ (N02)
    { "lane.csv",    N03_Lane },   // レーンデータ個別 (N03)
    { "wayarea.csv", N04_Wayarea } // 走行エリアデータ (N04)
  };
  const int ADAS_BASIC_COUNT = 10;  // 基本図形クラス／道路データの総数

  // 地物データファイルの対応
  const struct
  {
    std::string IDTEXT;   // 識別子文字列
    AdasKind_e  IDVALUE;  // 識別子内部値
  }
  ADAS_FEATURE_ITEMS[] =
  {
    { "R001", R001_Roadedge },     // 道路縁 (R001)
    { "R002", R002_Gutter },       // 側溝 (R002)
    { "R003", R003_Curb },         // 縁石 (R003)
    { "R004", R004_Intersection }, // 交差点 (R004)
    { "R005", R005_Sidestrip },    // 路肩コンクリート (R005)
    { "P001", P001_Whiteline },    // 白線 (P001)
    { "P002", P002_Stopline },     // 停止線 (P002)
    { "P003", P003_Zebrazone },    // ゼブラゾーン (P003)
    { "P004", P004_Crosswalk },    // 横断歩道 (P004)
    { "P005", P005_Roadmark },     // 路面マーク (P005)
    { "S001", S001_Guardrail },    // ガードレール (S001)
    { "S002", S002_Sidewalk },     // 歩道 (S002)
    { "S003", S003_Driveon },      // 車両乗入部 (S003)
    { "K001", K001_Poledata },     // ポール (K001)
    { "K002", K002_Utilitypole },  // 電柱 (K002)
    { "K003", K003_Roadsign },     // 標識 (K003)
    { "K004", K004_Signaldata },   // 信号 (K004)
    { "K005", K005_Streetlight },  // 街灯 (K005)
    { "K006", K006_Curvemirror },  // カーブミラー (K006)
    { "K007", K007_Wall },         // 壁面(K007)
    { "K008", K008_Fence },        // フェンス (K008)
    { "K009", K009_Railroad }      // 踏切ゾーン(K009)
  };
  const int ADAS_FEATURE_COUNT = 22;  // 地物データの総数
}

/************************************************************
*  インデックスファイル(idx.csv)の書き込み
*    format: Id,Kind,fname
*
*    input:  fname - ファイル名
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool writeAdasIdxCsv(std::string fname)
{
  // ファイルのオープン
  std::ofstream ofs(fname.c_str());
  if(!ofs)
  {
    std::cerr << "WARNING:[writeAdasIdxCsv-01] missing file: " << fname << std::endl;
    return false;
  }

  // ヘッダの書き込み
  ofs << "ID,KIND,fname" << std::endl;

  // 本体データの書き込み
  int i = 1;
  for(MpAdIndex_t::const_iterator it=g_MpAdIndex.begin(); it != g_MpAdIndex.end(); it++)
    ofs << i++ << "," << it->first << "," << it->second << std::endl;

  return true;
}

/************************************************************
*  データファイル(xxx.csv)の書き込み
*
*    input:  fname - ファイル名
*            fkind - ファイル種類
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool writeAdasDataCsv(std::string fname, AdasKind_e fkind)
{
  // データが空であれば、ファイルを作らない
  if(((fkind == B01_Point) && g_MpAdPoint.empty()) ||
     ((fkind == B02_Vector) && g_MpAdVector.empty()) ||
     ((fkind == B03_Pole) && g_MpAdPole.empty()) ||
     ((fkind == B04_Line) && g_MpAdLine.empty()) ||
     ((fkind == B05_Area) && g_MpAdArea.empty()) ||
     ((fkind == B06_Box) && g_MpAdBox.empty()) ||
     ((fkind == N01_Dtlane) && g_MpAdDtlane.empty()) ||
     ((fkind == N02_Node) && g_MpAdNode.empty()) ||
     ((fkind == N03_Lane) && g_MpAdLane.empty()) ||
     ((fkind == N04_Wayarea) && g_MpAdWayarea.empty()) ||
     ((fkind == R001_Roadedge) && g_MpAdRoadedge.empty()) ||
     ((fkind == R002_Gutter) && g_MpAdGutter.empty()) ||
     ((fkind == R003_Curb) && g_MpAdCurb.empty()) ||
     ((fkind == R004_Intersection) && g_MpAdIntersection.empty()) ||
     ((fkind == R005_Sidestrip) && g_MpAdSidestrip.empty()) ||
     ((fkind == P001_Whiteline) && g_MpAdWhiteline.empty()) ||
     ((fkind == P002_Stopline) && g_MpAdStopline.empty()) ||
     ((fkind == P003_Zebrazone) && g_MpAdZebrazone.empty()) ||
     ((fkind == P004_Crosswalk) && g_MpAdCrosswalk.empty()) ||
     ((fkind == P005_Roadmark) && g_MpAdRoadmark.empty()) ||
     ((fkind == S001_Guardrail) && g_MpAdGuardrail.empty()) ||
     ((fkind == S002_Sidewalk) && g_MpAdSidewalk.empty()) ||
     ((fkind == S003_Driveon) && g_MpAdDriveon.empty()) ||
     ((fkind == K001_Poledata) && g_MpAdPoledata.empty()) ||
     ((fkind == K002_Utilitypole) && g_MpAdUtilitypole.empty()) ||
     ((fkind == K003_Roadsign) && g_MpAdRoadsign.empty()) ||
     ((fkind == K004_Signaldata) && g_MpAdSignaldata.empty()) ||
     ((fkind == K005_Streetlight) && g_MpAdStreetlight.empty()) ||
     ((fkind == K006_Curvemirror) && g_MpAdCurvemirror.empty()) ||
     ((fkind == K007_Wall) && g_MpAdWall.empty()) ||
     ((fkind == K008_Fence) && g_MpAdFence.empty()) ||
     ((fkind == K009_Railroad) && g_MpAdRailroad.empty()))
    return true;

  // ファイルのオープン
  std::ofstream ofs(fname.c_str(), std::ios::trunc);
  if(!ofs)
  {
    std::cerr << "WARNING:[writeAdasDataCsv-01] cannot open file[" << fkind << "]: " << fname << std::endl;
    return false;
  }

  // データの書き込み
  if(fkind == B01_Point)
  {
    ofs << "PID,B,L,H,Bx,Ly,Ref,MCODE1,MCODE2,MCODE3" << std::endl;  // ヘッダの書き込み
    for(MpAdPoint_t::const_iterator it=g_MpAdPoint.begin(); it != g_MpAdPoint.end(); it++)
      ofs << it->first << ","
          << convDegToDms(it->second.B, true) << ","
          << convDegToDms(it->second.L, true) << ","
          << std::fixed << std::setprecision(3) << it->second.H << ","
            << it->second.Bx << ","
            << it->second.Ly << ","
          << it->second.Ref << ","
          << it->second.MCODE1 << ","
          << it->second.MCODE2 << ","
          << it->second.MCODE3 << std::endl;
  }
  else if(fkind == B02_Vector)
  {
    ofs << "VID,PID,Hang,Vang" << std::endl;
    for(MpAdVector_t::const_iterator it=g_MpAdVector.begin(); it != g_MpAdVector.end(); it++)
      ofs << it->first << ","
          << it->second.PID << ","
          << convDegToDms(it->second.Hang, false) << ","
          << convDegToDms(it->second.Vang, false) << std::endl;
  }
  else if(fkind == B03_Pole)
  {
    ofs << "PLID,VID,Length,Dim" << std::endl;
    for(MpAdPole_t::const_iterator it=g_MpAdPole.begin(); it != g_MpAdPole.end(); it++)
      ofs << it->first << ","
          << it->second.VID << ","
          << std::fixed << std::setprecision(3) << it->second.Length << ","
            << it->second.Dim << std::endl;
  }
  else if(fkind == B04_Line)
  {
    ofs << "LID,BPID,FPID,BLID,FLID" << std::endl;
    for(MpAdLine_t::const_iterator it=g_MpAdLine.begin(); it != g_MpAdLine.end(); it++)
      ofs << it->first << ","
          << it->second.BPID << ","
          << it->second.FPID << ","
          << it->second.BLID << ","
          << it->second.FLID << std::endl;
  }
  else if(fkind == B05_Area)
  {
    ofs << "AID,SLID,ELID" << std::endl;
    for(MpAdArea_t::const_iterator it=g_MpAdArea.begin(); it != g_MpAdArea.end(); it++)
      ofs << it->first << ","
          << it->second.SLID << ","
          << it->second.ELID << std::endl;
  }
  else if(fkind == B06_Box)
  {
    ofs << "BID,PID1,PID2,PID3,PID4,Height" << std::endl;
    for(MpAdBox_t::const_iterator it=g_MpAdBox.begin(); it != g_MpAdBox.end(); it++)
      ofs << it->first << ","
          << it->second.PID1 << ","
          << it->second.PID2 << ","
          << it->second.PID3 << ","
          << it->second.PID4 << ","
          << std::fixed << std::setprecision(3) << it->second.Height << std::endl;
  }
  else if(fkind == N01_Dtlane)
  {
    ofs << "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW" << std::endl;
    for(MpAdDtlane_t::const_iterator it=g_MpAdDtlane.begin(); it != g_MpAdDtlane.end(); it++)
      ofs << it->first << ","
          << std::fixed << std::setprecision(3) << it->second.Dist << ","
          << it->second.PID << ","
          << std::fixed << std::setprecision(8) << it->second.Dir << ","
          << std::fixed << std::setprecision(4) << it->second.Apara << ","
            << it->second.r << ","
            << it->second.slope << ","
            << it->second.cant << ","
          << std::fixed << std::setprecision(3) << it->second.LW << ","
            << it->second.RW << std::endl;
  }
  else if(fkind == N02_Node)
  {
    ofs << "NID,PID" << std::endl;
    for(MpAdNode_t::const_iterator it=g_MpAdNode.begin(); it != g_MpAdNode.end(); it++)
      ofs << it->first << ","
          << it->second.PID << std::endl;
  }
  else if(fkind == N03_Lane)
  {
    ofs << "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,FLID2,FLID3,FLID4,CrossID,"
        << "Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,LinkWAID" << std::endl;
    for(MpAdLane_t::const_iterator it=g_MpAdLane.begin(); it != g_MpAdLane.end(); it++)
      ofs << it->first << ","
          << it->second.DID << ","
          << it->second.BLID << ","
          << it->second.FLID << ","
          << it->second.BNID << ","
          << it->second.FNID << ","
          << it->second.JCT << ","
          << it->second.BLID2 << ","
          << it->second.BLID3 << ","
          << it->second.BLID4 << ","
          << it->second.FLID2 << ","
          << it->second.FLID3 << ","
          << it->second.FLID4 << ","
          << it->second.CrossID << ","
          << std::fixed << std::setprecision(3) << it->second.Span << ","
          << it->second.LCnt << ","
          << it->second.Lno << ","
          << it->second.LaneType << ","
          << it->second.LimitVel << ","
          << it->second.RefVel << ","
          << it->second.RoadSecID << ","
          << it->second.LaneChgFG << ","
          << it->second.LinkWAID << std::endl;
  }
  else if(fkind == N04_Wayarea)
  {
    ofs << "WAID,AID" << std::endl;
    for(MpAdWayarea_t::const_iterator it=g_MpAdWayarea.begin(); it != g_MpAdWayarea.end(); it++)
      ofs << it->first << ","
          << it->second.AID << std::endl;
  }
  else if(fkind == R001_Roadedge)
  {
    ofs << "ID,LID,LinkID" << std::endl;
    for(MpAdRoadedge_t::const_iterator it=g_MpAdRoadedge.begin(); it != g_MpAdRoadedge.end(); it++)
      ofs << it->first << ","
          << it->second.LID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == R002_Gutter)
  {
    ofs << "ID,AID,Type,LinkID" << std::endl;
    for(MpAdGutter_t::const_iterator it=g_MpAdGutter.begin(); it != g_MpAdGutter.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.Type << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == R003_Curb)
  {
    ofs << "ID,LID,Height,Width,Dir,LinkID" << std::endl;
    for(MpAdCurb_t::const_iterator it=g_MpAdCurb.begin(); it != g_MpAdCurb.end(); it++)
      ofs << it->first << ","
          << it->second.LID << ","
          << std::fixed << std::setprecision(3) << it->second.Height << ","
          << std::fixed << std::setprecision(3) << it->second.Width << ","
          << it->second.Dir << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == R004_Intersection)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdIntersection_t::const_iterator it=g_MpAdIntersection.begin(); it != g_MpAdIntersection.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == R005_Sidestrip)
  {
    ofs << "ID,LID,LinkID" << std::endl;
    for(MpAdSidestrip_t::const_iterator it=g_MpAdSidestrip.begin(); it != g_MpAdSidestrip.end(); it++)
      ofs << it->first << ","
          << it->second.LID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == P001_Whiteline)
  {
    ofs << "ID,LID,Width,Color,type,LinkID" << std::endl;
    for(MpAdWhiteline_t::const_iterator it=g_MpAdWhiteline.begin(); it != g_MpAdWhiteline.end(); it++)
      ofs << it->first << ","
          << it->second.LID << ","
          << std::fixed << std::setprecision(3) << it->second.Width << ","
          << it->second.Color << ","
          << it->second.Type << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == P002_Stopline)
  {
    ofs << "ID,LID,TLID,SignID,LinkID" << std::endl;
    for(MpAdStopline_t::const_iterator it=g_MpAdStopline.begin(); it != g_MpAdStopline.end(); it++)
      ofs << it->first << ","
          << it->second.LID << ","
          << it->second.TLID << ","
          << it->second.SignID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == P003_Zebrazone)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdZebrazone_t::const_iterator it=g_MpAdZebrazone.begin(); it != g_MpAdZebrazone.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == P004_Crosswalk)
  {
    ofs << "ID,AID,Type,BdID,LinkID" << std::endl;
    for(MpAdCrosswalk_t::const_iterator it=g_MpAdCrosswalk.begin(); it != g_MpAdCrosswalk.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.Type << ","
          << it->second.BdID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == P005_Roadmark)
  {
    ofs << "ID,AID,Type,LinkID" << std::endl;
    for(MpAdRoadmark_t::const_iterator it=g_MpAdRoadmark.begin(); it != g_MpAdRoadmark.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.Type << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == S001_Guardrail)
  {
    ofs << "ID,AID,Type,LinkID" << std::endl;
    for(MpAdGuardrail_t::const_iterator it=g_MpAdGuardrail.begin(); it != g_MpAdGuardrail.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.Type << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == S002_Sidewalk)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdSidewalk_t::const_iterator it=g_MpAdSidewalk.begin(); it != g_MpAdSidewalk.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == S003_Driveon)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdDriveon_t::const_iterator it=g_MpAdDriveon.begin(); it != g_MpAdDriveon.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K001_Poledata)
  {
    ofs << "ID,PLID,LinkID" << std::endl;
    for(MpAdPoledata_t::const_iterator it=g_MpAdPoledata.begin(); it != g_MpAdPoledata.end(); it++)
      ofs << it->first << ","
          << it->second.PLID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K002_Utilitypole)
  {
    ofs << "ID,PLID,LinkID" << std::endl;
    for(MpAdUtilitypole_t::const_iterator it=g_MpAdUtilitypole.begin(); it != g_MpAdUtilitypole.end(); it++)
      ofs << it->first << ","
          << it->second.PLID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K003_Roadsign)
  {
    ofs << "ID,VID,PLID,Type,LinkID" << std::endl;
    for(MpAdRoadsign_t::const_iterator it=g_MpAdRoadsign.begin(); it != g_MpAdRoadsign.end(); it++)
      ofs << it->first << ","
          << it->second.VID << ","
          << it->second.PLID << ","
          << it->second.Type << ","
         << it->second.LinkID << std::endl;
  }
  else if(fkind == K004_Signaldata)
  {
    ofs << "ID,VID,PLID,Type,LinkID" << std::endl;
    for(MpAdSignaldata_t::const_iterator it=g_MpAdSignaldata.begin(); it != g_MpAdSignaldata.end(); it++)
      ofs << it->first << ","
          << it->second.VID << ","
          << it->second.PLID << ","
          << it->second.Type << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K005_Streetlight)
  {
    ofs << "ID,LID,PLID,LinkID" << std::endl;
    for(MpAdStreetlight_t::const_iterator it=g_MpAdStreetlight.begin(); it != g_MpAdStreetlight.end(); it++)
      ofs << it->first << ","
          << it->second.LID << ","
          << it->second.PLID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K006_Curvemirror)
  {
    ofs << "ID,VID,PLID,Type,LinkID" << std::endl;
    for(MpAdCurvemirror_t::const_iterator it=g_MpAdCurvemirror.begin(); it != g_MpAdCurvemirror.end(); it++)
      ofs << it->first << ","
          << it->second.VID << ","
          << it->second.PLID << ","
          << it->second.Type << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K007_Wall)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdWall_t::const_iterator it=g_MpAdWall.begin(); it != g_MpAdWall.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K008_Fence)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdFence_t::const_iterator it=g_MpAdFence.begin(); it != g_MpAdFence.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }
  else if(fkind == K009_Railroad)
  {
    ofs << "ID,AID,LinkID" << std::endl;
    for(MpAdRailroad_t::const_iterator it=g_MpAdRailroad.begin(); it != g_MpAdRailroad.end(); it++)
      ofs << it->first << ","
          << it->second.AID << ","
          << it->second.LinkID << std::endl;
  }

  return true;
}

/************************************************************
*  ADASマップファイルの書き込み
*
*    input: dname - ディレクトリ名
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
bool writeAdasMapFiles(std::string dname)
{
  std::string fname;  // ファイル名バッファ
#ifndef _QT
  struct stat st;
#endif

  if(!dname.empty())
  {
    // ディレクトリの存在確認
#ifdef _QT
    QDir adir(QString::fromStdString(dname));
    if(!adir.exists())
    {
      if(QFile::exists(QString::fromStdString(dname)))
      {
        std::cerr << "WARNING:[writeAdasMapFiles-01] same name file already exist: " << dname << std::endl;
        return false;
      }
      // ディレクトリもファイルも無いので、新規ディレクトリ作成
      if(!adir.mkpath("."))
      {
        std::cerr << "WARNING:[writeAdasMapFiles-02] cannot make directory: " << dname << std::endl;
        return false;
      }
      //std::cerr << "DEBUG:[writeAdasMapFiles] make directory: " << dname << std::endl;
      std::cout << " make new ADAS directory: " << dname << std::endl;

    }
#else
    if(stat(dname.c_str(), &st))
    {
      // ディレクトリもファイルも無いので、新規ディレクトリ作成
      if(mkdir(dname.c_str(), (S_IRWXU | S_IRWXG | S_IRWXO)))
      {
        std::cerr << "WARNING:[writeAdasMapFiles-03] cannot make directory: " << dname << std::endl;
        return false;
      }
      //std::cerr << "DEBUG:[writeAdasMapFiles] make directory: " << dname << std::endl;
      std::cout << " make new ADAS directory: " << dname << std::endl;
    }
    else
    {
      if(!S_ISDIR(st.st_mode))
      {
        std::cerr << "WARNING:[writeAdasMapFiles-04] same name file already exist: " << dname << std::endl;
        return false;
      }
    }
#endif

    // ディレクトリとファイルの区切りを付加
    if(dname[dname.size() - 1] != '/')
      dname += "/";
  }

  // 基本図形クラス／道路データファイルの書き込み
  for(int idx=0; idx<ADAS_BASIC_COUNT; idx++)
  {
    if(!writeAdasDataCsv(dname + ADAS_BASIC_ITEMS[idx].FNAME, ADAS_BASIC_ITEMS[idx].IDVALUE))
    {
      std::cerr << "WARNING:[writeAdasMapFiles-05] " << ADAS_BASIC_ITEMS[idx].FNAME
                << " write error" << std::endl;
      return false;
    }
  }

  // 地物データファイルの書き込み
  insertAdasIndex("R001", "roadedge.csv");
  insertAdasIndex("R002", "gutter.csv");
  insertAdasIndex("R003", "curb.csv");
  insertAdasIndex("R004", "intersection.csv");
  insertAdasIndex("R005", "sidestrip.csv");
  insertAdasIndex("P001", "whiteline.csv");
  insertAdasIndex("P002", "stopline.csv");
  insertAdasIndex("P003", "zebrazone.csv");
  insertAdasIndex("P004", "crosswalk.csv");
  insertAdasIndex("P005", "road_surface_mark.csv");
  insertAdasIndex("S001", "guardrail.csv");
  insertAdasIndex("S002", "sidewalk.csv");
  insertAdasIndex("S003", "driveon_portion.csv");
  insertAdasIndex("K001", "poledata.csv");
  insertAdasIndex("K002", "utilitypole.csv");
  insertAdasIndex("K003", "roadsign.csv");
  insertAdasIndex("K004", "signaldata.csv");
  insertAdasIndex("K005", "streetlight.csv");
  insertAdasIndex("K006", "curvemirror.csv");
  insertAdasIndex("K007", "wall.csv");
  insertAdasIndex("K008", "fence.csv");
  insertAdasIndex("K009", "railroad_crossing.csv");

  for(int idx=0; idx<ADAS_FEATURE_COUNT; idx++)
  {
    fname = g_MpAdIndex[ADAS_FEATURE_ITEMS[idx].IDTEXT];
    if(!fname.empty())
    {
      if(!writeAdasDataCsv(dname + fname, ADAS_FEATURE_ITEMS[idx].IDVALUE))
      {
        std::cerr << "WARNING:[writeAdasMapFiles-06] " << fname << "write error" << std::endl;
        return false;
      }
    }
  }

  // インデクスファイルの書き込み
  if(!writeAdasIdxCsv(dname + "idx.csv"))
  {
    std::cerr << "WARNING:[writeAdasMapFiles-07] idx.csv write error" << std::endl;
    return false;
  }

  return true;
}

/************************************************************
*  ADAS各種データのバッファへの追加
*
*    input: 追加するパラメータ
*    output: nothing
*    return: nothing
************************************************************/

// インデックスデータの追加
void insertAdasIndex(std::string KIND, std::string fname)
{
  g_MpAdIndex.insert(std::make_pair(KIND, fname));
}

// ポイントデータ(B01)の追加
void insertAdasPoint(int PID, double B, double L, double H, double Bx, double Ly,
                     int Ref, int MCODE1, int MCODE2, int MCODE3)
{
  StAdPoint_t stbuff;
  stbuff.B = B;
  stbuff.L = L;
  stbuff.H = H;
  stbuff.Bx = Bx;
  stbuff.Ly = Ly;
  stbuff.Ref = Ref;
  stbuff.MCODE1 = MCODE1;
  stbuff.MCODE2 = MCODE2;
  stbuff.MCODE3 = MCODE3;
  g_MpAdPoint.insert(std::make_pair(PID, stbuff));
}

// ベクトルデータ(B02)の追加
void insertAdasVector(int VID, int PID, double Hang, double Vang)
{
  StAdVector_t stbuff;
  stbuff.PID = PID;
  stbuff.Hang = Hang;
  stbuff.Vang = Vang;
  g_MpAdVector.insert(std::make_pair(VID, stbuff));
}

// ポールデータ(B03)の追加
void insertAdasPole(int PLID, int VID, double Length, double Dim)
{
  StAdPole_t stbuff;
  stbuff.VID = VID;
  stbuff.Length = Length;
  stbuff.Dim = Dim;
  g_MpAdPole.insert(std::make_pair(PLID, stbuff));
}

// ラインデータ(B04)の追加
void insertAdasLine(int LID, int BPID, int FPID, int BLID, int FLID)
{
  StAdLine_t stbuff;
  stbuff.BPID = BPID;
  stbuff.FPID = FPID;
  stbuff.BLID = BLID;
  stbuff.FLID = FLID;
  g_MpAdLine.insert(std::make_pair(LID, stbuff));
}

// エリアデータ(B05)の追加
void insertAdasArea(int AID, int SLID, int ELID)
{
  StAdArea_t stbuff;
  stbuff.SLID = SLID;
  stbuff.ELID = ELID;
  g_MpAdArea.insert(std::make_pair(AID, stbuff));
}

// ボックスデータ(B06)の追加
void insertAdasBox(int BID, int PID1, int PID2, int PID3, int PID4, double Height)
{
  StAdBox_t stbuff;
  stbuff.PID1 = PID1;
  stbuff.PID2 = PID2;
  stbuff.PID3 = PID3;
  stbuff.PID4 = PID4;
  stbuff.Height = Height;
  g_MpAdBox.insert(std::make_pair(BID, stbuff));
}

// 中心線形データ(N01)の追加
void insertAdasDtlene(int DID, double Dist, int PID, double Dir, double Apara, double r,
                      double slope, double cant, double LW, double RW)
{
  StAdDtlane_t stbuff;
  stbuff.Dist = Dist;
  stbuff.PID = PID;
  stbuff.Dir = Dir;
  stbuff.Apara = Apara;
  stbuff.r = r;
  stbuff.slope = slope;
  stbuff.cant = cant;
  stbuff.LW = LW;
  stbuff.RW = RW;
  g_MpAdDtlane.insert(std::make_pair(DID, stbuff));
}

// ノードデータ(N02)の追加
void insertAdasNode(int NID, int PID)
{
  StAdNode_t stbuff;
  stbuff.PID = PID;
  g_MpAdNode.insert(std::make_pair(NID, stbuff));
}

// レーンデータ(N03)の追加
void insertAdasLane(int LnID, int DID, int BLID, int FLID, int BNID, int FNID, int JCT,
                    int BLID2, int BLID3, int BLID4, int FLID2, int FLID3, int FLID4,
                    int CrossID, double Span, int LCnt, int Lno,
                    int LaneType, int LimitVel,int RefVel, int RoadSecID, int LaneChgFG, int LinkWAID)
{
  StAdLane_t stbuff;
  stbuff.DID  = DID;
  stbuff.BLID = BLID;
  stbuff.FLID = FLID;
  stbuff.BNID = BNID;
  stbuff.FNID = FNID;
  stbuff.JCT  = JCT;
  stbuff.BLID2 = BLID2;
  stbuff.BLID3 = BLID3;
  stbuff.BLID4 = BLID4;
  stbuff.FLID2 = FLID2;
  stbuff.FLID3 = FLID3;
  stbuff.FLID4 = FLID4;
  stbuff.CrossID = CrossID;
  stbuff.Span = Span;
  stbuff.LCnt = LCnt;
  stbuff.Lno  = Lno;
  stbuff.LaneType = LaneType;
  stbuff.LimitVel = LimitVel;
  stbuff.RefVel = RefVel;
  stbuff.RoadSecID = RoadSecID;
  stbuff.LaneChgFG = LaneChgFG;
  stbuff.LinkWAID = LinkWAID;
  g_MpAdLane.insert(std::make_pair(LnID, stbuff));
}

// 走行エリア(N04)の追加
void insertAdasWayarea(int WAID, int AID)
{
  StAdWayarea_t stbuff;
  stbuff.AID = AID;
  g_MpAdWayarea.insert(std::make_pair(WAID, stbuff));
}

// 道路縁(R001)の追加
void insertAdasRoadedge(int ID, int LID, int LinkID)
{
  StAdRoadedge_t stbuff;
  stbuff.LID = LID;
  stbuff.LinkID = LinkID;
  g_MpAdRoadedge.insert(std::make_pair(ID, stbuff));
}

// 側溝(R002)の追加
void insertAdasGutter(int ID, int AID, int Type, int LinkID)
{
  StAdGutter_t stbuff;
  stbuff.AID = AID;
  stbuff.Type = Type;
  stbuff.LinkID = LinkID;
  g_MpAdGutter.insert(std::make_pair(ID, stbuff));
}

// 縁石(R003)の追加
void insertAdasCurb(int ID, int LID, double Height, double Width, int Dir, int LinkID)
{
  StAdCurb_t stbuff;
  stbuff.LID = LID;
  stbuff.Height = Height;
  stbuff.Width = Width;
  stbuff.Dir = Dir;
  stbuff.LinkID = LinkID;
  g_MpAdCurb.insert(std::make_pair(ID, stbuff));
}

// 交差点(R004)の追加
void insertAdasIntersection(int ID, int AID, int LinkID)
{
  StAdIntersection_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdIntersection.insert(std::make_pair(ID, stbuff));
}
// 路肩コンクリート(R005)の追加
void insertAdasSidestrip(int ID, int LID, int LinkID)
{
  StAdSidestrip_t stbuff;
  stbuff.LID = LID;
  stbuff.LinkID = LinkID;
  g_MpAdSidestrip.insert(std::make_pair(ID, stbuff));
}

// 白線(P001)の追加
void insertAdasWhiteline(int ID, int LID, double Width, std::string Color, int type, int LinkID)
{
  StAdWhiteline_t stbuff;
  stbuff.LID = LID;
  stbuff.Width = Width;
  stbuff.Color = Color;
  stbuff.Type = type;
  stbuff.LinkID = LinkID;
  g_MpAdWhiteline.insert(std::make_pair(ID, stbuff));
}

// 停止線(P002)の追加
void insertAdasStopline(int ID, int LID, int TLID, int SignID, int LinkID)
{
  StAdStopline_t stbuff;
  stbuff.LID = LID;
  stbuff.TLID = TLID;
  stbuff.SignID = SignID;
  stbuff.LinkID = LinkID;
  g_MpAdStopline.insert(std::make_pair(ID, stbuff));
}

// ゼブラゾーン(P003)の追加
void insertAdasZebrazone(int ID, int AID, int LinkID)
{
  StAdZebrazone_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdZebrazone.insert(std::make_pair(ID, stbuff));
}

// 横断歩道(P004)の追加
void insertAdasCrosswalk(int ID, int AID, int Type, int BdID, int LinkID)
{
  StAdCrosswalk_t stbuff;
  stbuff.AID = AID;
  stbuff.Type = Type;
  stbuff.BdID = BdID;
  stbuff.LinkID = LinkID;
  g_MpAdCrosswalk.insert(std::make_pair(ID, stbuff));
}

// 路面マーク(P005)の追加
void insertAdasRoadmark(int ID, int AID, std::string Type, int LinkID)
{
  StAdRoadmark_t stbuff;
  stbuff.AID = AID;
  stbuff.Type = Type;
  stbuff.LinkID = LinkID;
  g_MpAdRoadmark.insert(std::make_pair(ID, stbuff));
}

// ガードレール(S001)の追加
void insertAdasGuardrail(int ID, int AID, int Type, int LinkID)
{
  StAdGuardrail_t stbuff;
  stbuff.AID = AID;
  stbuff.Type = Type;
  stbuff.LinkID = LinkID;
  g_MpAdGuardrail.insert(std::make_pair(ID, stbuff));
}

// 歩道(S002)の追加
void insertAdasSidewalk(int ID, int AID, int LinkID)
{
  StAdSidewalk_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdSidewalk.insert(std::make_pair(ID, stbuff));
}

// 車両乗入部(S003)の追加
void insertAdasDriveon(int ID, int AID, int LinkID)
{
  StAdDriveon_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdDriveon.insert(std::make_pair(ID, stbuff));
}

// ポール(K001)の追加
void insertAdasPoledata(int ID, int PLID, int LinkID)
{
  StAdPoledata_t stbuff;
  stbuff.PLID = PLID;
  stbuff.LinkID = LinkID;
  g_MpAdPoledata.insert(std::make_pair(ID, stbuff));
}

// 電柱(K002)の追加
void insertAdasUtilitypole(int ID, int PLID, int LinkID)
{
  StAdUtilitypole_t stbuff;
  stbuff.PLID = PLID;
  stbuff.LinkID = LinkID;
  g_MpAdUtilitypole.insert(std::make_pair(ID, stbuff));
}

// 標識(K003)の追加
void insertAdasRoadsign(int ID, int VID, int PLID, std::string Type, int LinkID)
{
  StAdRoadsign_t stbuff;
  stbuff.VID  = VID;
  stbuff.PLID = PLID;
  stbuff.Type = Type;
  stbuff.LinkID = LinkID;
  g_MpAdRoadsign.insert(std::make_pair(ID, stbuff));
}

// 信号(K004)の追加
void insertAdasSignaldata(int ID, int VID, int PLID, int Type, int LinkID)
{
  StAdSignaldata_t stbuff;
  stbuff.VID = VID;
  stbuff.PLID = PLID;
  stbuff.Type = Type;
  stbuff.LinkID = LinkID;
  g_MpAdSignaldata.insert(std::make_pair(ID, stbuff));
}

// 街灯(K005)の追加
void insertAdasStreetlight(int ID, int LID, int PLID, int LinkID)
{
  StAdStreetlight_t stbuff;
  stbuff.LID = LID;
  stbuff.PLID = PLID;
  stbuff.LinkID = LinkID;
  g_MpAdStreetlight.insert(std::make_pair(ID, stbuff));
}

// カーブミラー(K006)の追加
void insertAdasCurvemirror(int ID, int VID, int PLID, std::string Type, int LinkID)
{
  StAdCurvemirror_t stbuff;
  stbuff.VID = VID;
  stbuff.PLID = PLID;
  stbuff.Type = Type;
  stbuff.LinkID = LinkID;
  g_MpAdCurvemirror.insert(std::make_pair(ID, stbuff));
}

// 壁面(K007)の追加
void insertAdasWall(int ID, int AID, int LinkID)
{
  StAdWall_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdWall.insert(std::make_pair(ID, stbuff));
}

// フェンス(K008)の追加
void insertAdasFence(int ID, int AID, int LinkID)
{
  StAdFence_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdFence.insert(std::make_pair(ID, stbuff));
}

// 踏切ゾーン(K009)の追加
void insertAdasRailroad(int ID, int AID, int LinkID)
{
  StAdRailroad_t stbuff;
  stbuff.AID = AID;
  stbuff.LinkID = LinkID;
  g_MpAdRailroad.insert(std::make_pair(ID, stbuff));
}

