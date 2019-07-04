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
*  dmp_to_adas.cpp
*  データベース形式ダイナミックマップからADAS-MAPへの変換処理
****************************************/

#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

#include "dmp_data.h"
#include "dmp_file.h"
#include "adas_data.h"
#include "adas_file.h"
#include "commonlib.h"
#include "dmp_to_adas.h"

/************************************************************
*  グローバル変数定義
************************************************************/

extern bool bDebugFlg;        // デバッグログ出力フラグ
extern std::ofstream ofsdbg;  // デバッグログ出力ストリーム

/************************************************************
*  ローカル変数定義
************************************************************/
namespace
{
  // 拡張レーンバッファ
  typedef struct
  {
    int feature_id;      // Dmpレーン地物ID
    int sno;             // Dmpレーン内一連番号(0-)
    int ad_LnID;         // 対応するADASレーンID
    double xRct;         // 始点の平面直角系X座標 [m]
    double yRct;         // 始点の平面直角系Y座標 [m]
    double zHgt;         // 始点の標高 [m]
  } StLaneExBuff_t;
  typedef std::vector<StLaneExBuff_t> VcLaneExBuff_t;
  VcLaneExBuff_t mVcLaneExBuff;

  // レーン始点終点レーンバッファ
  typedef struct
  {
    int feature_id;      // Dmpレーン地物ID (キー)
    int bgnLnID;         // 始点ADASレーンID
    int endLnID;         // 終点ADASレーンID
  } StLaneBgnEndBuff_t;
  typedef std::map<int, StLaneBgnEndBuff_t> MpLaneBgnEndBuff_t;
  MpLaneBgnEndBuff_t mMpLaneBgnEndBuff;

  // 交差点領域拡張バッファ
  typedef struct  // X,Y座標バッファ
  {
    float xRct;  // 平面直角系X座標 [m]
    float yRct;  // 平面直角系Y座標 [m]
  } StXyPoint_t;

  typedef struct
  {
    int    IsID;  // 対応するADAS交差点ID
    double xmin;  // 交差点領域の最小平面直角系X座標 [m]
    double xmax;  // 交差点領域の最大平面直角系X座標 [m]
    double ymin;  // 交差点領域の最小平面直角系Y座標 [m]
    double ymax;  // 交差点領域の最大平面直角系Y座標 [m]
    std::vector<StXyPoint_t> pgn;  // 交差点領域の閉じた多角形
  } StIntersectionAreaExBuff_t;
  typedef std::vector<StIntersectionAreaExBuff_t> VcIntersectionAreaExBuff_t;
  VcIntersectionAreaExBuff_t mVcIntersectionAreaExBuff;

  // 停止線拡張バッファ
  typedef struct
  {
    int feature_id;     // Dmp停止線地物ID (キー)
    int ad_StoplineID;  // 対応するADAS停止線ID
    double xRct;        // 中点の平面直角系X座標 [m]
    double yRct;        // 中点の平面直角系Y座標 [m]
    double zHgt;        // 中点の標高 [m]
  } StStoplineExBuff_t;
  typedef std::map<int, StStoplineExBuff_t> MpStoplineExBuff_t;
  MpStoplineExBuff_t mMpStoplineExBuff;

  // 標識拡張バッファ
  typedef struct
  {
    int feature_id;     // Dmp標識地物ID (キー)
    int ad_RoadsignID;  // 対応するADAS標識ID
    double xRct;        // 平面直角系X座標 [m]
    double yRct;        // 平面直角系Y座標 [m]
    double zHgt;        // 標高 [m]
  } StRoadsignExBuff_t;
  typedef std::map<int, StRoadsignExBuff_t> MpRoadsignExBuff_t;
  MpRoadsignExBuff_t mMpRoadsignExBuff;

  // 信号拡張バッファ
  typedef struct  // 信号ランプ毎バッファ
  {
    int ad_SignaldataID;  // 対応するADAS信号ID
    double xRct;          // 平面直角系X座標 [m]
    double yRct;          // 平面直角系Y座標 [m]
    double zHgt;          // 標高 [m]
  } StSignalLampExBuff_t;
  typedef std::vector<StSignalLampExBuff_t> VcSignalLampExBuff_t;

  typedef struct
  {
    int feature_id;    // Dmp信号地物ID (キー)
    VcSignalLampExBuff_t ad_SignalLamps; // 対応するADAS信号ランプ群
  } StSignaldataExBuff_t;
  typedef std::map<int, StSignaldataExBuff_t> MpSignaldataExBuff_t;
  MpSignaldataExBuff_t mMpSignaldataExBuff;

  // ADASデータID
  int c_PID;   // 現在ポイントID (=登録数)(B01)
  int c_VID;   // 現在ベクトルID (=登録数)(B02)
  int c_PLID;  // 現在ポールID (=登録数)(B03)
  int c_LID;   // 現在ラインID (=登録数)(B04)
  int c_AID;   // 現在エリアID (=登録数)(B05)
  int c_BID;   // 現在ボックスID (=登録数)(B06)

  int c_DID;   // 現在中心線形レコードID (=登録数)(N01)
  int c_NID;   // 現在ノードID (=登録数)(N02)
  int c_LnID;  // 現在レーンID (=登録数)(N03)
  int c_WAID;  // 現在走行エリアID (=登録数)(N04)

  int c_RoadedgeID;     // 現在道路縁ID (=登録数)(R001)
  int c_GutterID;       // 現在側溝ID (=登録数)(R002)
  int c_CurbID;         // 現在縁石ID (=登録数)(R003)
  int c_IntersectionID;  // 現在交差点ID (=登録数)(R004)
  int c_SidestripID;    // 現在路肩コンクリートID (=登録数)(R005)

  int c_WhitelineID;    // 現在白線ID (=登録数)(P001)
  int c_StoplineID;     // 現在停止線ID (=登録数)(P002)
  int c_ZebrazoneID;    // 現在ゼブラゾーンID (=登録数)(P003)
  int c_CrosswalkID;    // 現在横断歩道ID (=登録数)(P004)
  int c_RoadmarkID;     // 現在路面マークID (=登録数)(P005)

  int c_GuardrailID;    // 現在ガードレールID (=登録数)(S001)
  int c_SidewalkID;     // 現在歩道ID (=登録数)(S002)
  int c_DriveonID;      // 現在車両乗入部ID (=登録数)(S003)
  int c_PoledataID;     // 現在ポールID (=登録数)(K001)
  int c_UtilitypoleID;  // 現在電柱ID (=登録数)(K002)
  int c_RoadsignID;     // 現在標識ID (=登録数)(K003)
  int c_SignaldataID;   // 現在信号ID (=登録数)(K004)
  int c_StreetlightID;  // 現在街灯ID (=登録数)(K005)
  int c_CurvemirrorID;  // 現在カーブミラーID (=登録数)(K006)
  int c_WallID;         // 現在壁面ID (=登録数)(K007)
  int c_FenceID;        // 現在フェンスID (=登録数)(K008)
  int c_RailroadID;     // 現在踏切ゾーンID (=登録数)(K009)
}

/************************************************************
*  点が多角形に内包されるかを調べる (交差点内にあるかの判定)
*
*    input: pgn - 閉じた多角形座標配列 (最終の点は、最初の点と同じ座標)
*           x - 点の平面直角系X座標 [m]
*           y - 点の平面直角系X座標 [m]
*    output: nothing
*    return: 結果 (true=内包される、false=内包されない)
************************************************************/
static bool checkInclusion(std::vector<StXyPoint_t> pgn, double x, double y)
{
  double ang = 0;
  int sz = (int)pgn.size() - 1;

  for(int i=0; i<sz; i++)
  {
    double ax = pgn[i].xRct - x;
    double ay = pgn[i].yRct - y;
    double bx = pgn[i + 1].xRct - x;
    double by = pgn[i + 1].yRct - y;

    double cos = (ax * bx + ay * by) / (sqrt(ax * ax + ay * ay) * sqrt(bx * bx + by * by));
    ang += acos(cos);
  }

  if(fabs(ang - M_PI*2) < 0.01)
    return true;

  return false;
}

/************************************************************
*  ADASノードデータの座標を取得する
*
*    input: nid - ノードID
*    output: x - ノードの平面直角座標系 X座標 [m] (北向き)
*            y - ノードの平面直角座標系 Y座標 [m] (東向き)
*            z - ノードの標高 [m]
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool getAdasNodePoint(int nid, double& x, double& y, double& z)
{
  // ADASノードの検索
  MpAdNode_t::iterator itn = g_MpAdNode.find(nid);
  if(itn == g_MpAdNode.end())
  {
    std::cerr << "ERROR:[getAdasNodePoint-01] illegal nid=" << nid << std::endl;
    return false;
  }

  // ADASノードのポイントデータの検索
  MpAdPoint_t::iterator itp = g_MpAdPoint.find(itn->second.PID);
  if(itp == g_MpAdPoint.end())
  {
    std::cerr << "ERROR:[getAdasNodePoint-02] illegal nid=" << nid << std::endl;
    return false;
  }

  x = itp->second.Bx;
  y = itp->second.Ly;
  z = itp->second.H;

  return true;
}

/************************************************************
*  ADASレーンデータの始点座標と終点座標を取得する
*
*    input: lnid - レーンID
*    output: x1 - 始点の平面直角座標系 X座標 [m] (北向き)
*            y1 - 始点の平面直角座標系 Y座標 [m] (東向き)
*            z1 - 始点の標高 [m]
*            x2 - 終点の平面直角座標系 X座標 [m] (北向き)
*            y2 - 終点の平面直角座標系 Y座標 [m] (東向き)
*            z2 - 終点の標高 [m]
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool getAdasLanePoints(int lnid, double& x1, double& y1, double& z1,
                              double& x2, double& y2, double& z2)
{
  // ADASレーンの検索
  MpAdLane_t::iterator itl = g_MpAdLane.find(lnid);
  if(itl == g_MpAdLane.end())
  {
    std::cerr << "ERROR:[getAdasLanePoints-01] illegal lnid=" << lnid << std::endl;
    return false;
  }

  // ADASレーンの始点ノードの座標を取得
  if(!getAdasNodePoint(itl->second.BNID, x1, y1, z1))
  {
    std::cerr << "ERROR:[getAdasLanePoints-02] illegal lnid=" << lnid << ", BNID=" << itl->second.BNID << std::endl;
    return false;
  }

  // ADASレーンの終点ノードの座標を取得
  if(!getAdasNodePoint(itl->second.FNID, x2, y2, z2))
  {
    std::cerr << "ERROR:[getAdasLanePoints-03] illegal lnid=" << lnid << ", FNID=" << itl->second.FNID << std::endl;
    return false;
  }

  return true;
}

/************************************************************
*  ADASレーンデータの前後の角度を調べる
*
*    input: lnid1 - 出発地となるレーンID
*           lnid2 - 目的地となるレーンID
*    output: nothing
*    return: 曲がり角 [rad](-π~π、右が+、左が-)
************************************************************/
static double calAngleCrossAdasLanes(int lnid1, int lnid2)
{
  // 出発地ADASレーンの座標取得
  double bx1, by1, bz1, bx2, by2, bz2;
  if(!getAdasLanePoints(lnid1, bx1, by1, bz1, bx2, by2, bz2))
  {
    std::cerr << "ERROR:[calAngleCrossAdasLanes-01] illegal lnid1=" << lnid1 << std::endl;
    return M_PI;
  }

  // 目的地ADASレーンの座標取得
  double fx1, fy1, fz1, fx2, fy2, fz2;
  if(!getAdasLanePoints(lnid2, fx1, fy1, fz1, fx2, fy2, fz2))
  {
    std::cerr << "ERROR:[calAngleCrossLanes-02] illegal lnid2=" << lnid2 << std::endl;
    return M_PI;
  }

  // 方向角の差の計算 [rad]
  double ang = atan2(fx2 - fx1, fy2 - fy1) - atan2(bx2 - bx1, by2 - by1);
  if(ang <= -M_PI)
    ang += M_PI * 2;
  else if(ang > M_PI)
    ang -= M_PI * 2;

  return ang;
}

/************************************************************
*  点の、最寄りのADASレーンIDを調べる
*
*    output: cx - 点の平面直角座標系 X座標 [m]
*            cy - 点の平面直角座標系 Y座標 [m]
*            cz - 点の標高 [m]
*    output: nothing
*    return: 最寄りのADASレーンID (ないとき0)
************************************************************/
static int searchAdasLaneNearestToPoint(double cx, double cy, double cz)
{
  int result = 0;  // 結果(最寄りのADASレーンID)
  double dmin = 9.0E10;  // 距離

  // 車線リンク拡張レーンバッファの中で最寄りのものを検索
  for(VcLaneExBuff_t::const_iterator it = mVcLaneExBuff.begin();
      it != mVcLaneExBuff.end(); ++it)
  {
    double dx = cx - it->xRct;
    double dy = cy - it->yRct;
    double dz = cz - it->zHgt;
    double d = dx*dx + dy*dy + dz*dz;
    if(dmin > d)
    {
      result = it->ad_LnID;
      dmin = d;
    }
  }

  return result;
}

/************************************************************
*  Dmpレーン地物IDを指定して、点の、最寄りのADASレーンIDを調べる
*
*    output: cx - 点の平面直角座標系 X座標 [m]
*            cy - 点の平面直角座標系 Y座標 [m]
*            cz - 点の標高 [m]
*            fid - 変換元のDmpレーン地物ID
*    output: nothing
*    return: 最寄りのADASレーンID (ないとき0)
************************************************************/
static int searchAdasLaneNearestToPointWithFid(double cx, double cy, double cz, int fid)
{
  int result = 0;  // 結果(最寄りのADASレーンID)
  double dmin = 9.0E10;  // 距離

  // 車線リンク拡張レーンバッファの中で最寄りのものを検索
  for(VcLaneExBuff_t::const_iterator it = mVcLaneExBuff.begin();
      it != mVcLaneExBuff.end(); ++it)
  {
    if(it->feature_id == fid)
    {
      double dx = cx - it->xRct;
      double dy = cy - it->yRct;
      double dz = cz - it->zHgt;
      double d = dx*dx + dy*dy + dz*dz;
      if(dmin > d)
      {
        result = it->ad_LnID;
        dmin = d;
      }
    }
  }

  return result;
}


/************************************************************
*  一連のADASラインデータの、最寄りのADASレーンIDを調べる
*
*    input: slid - 最初のADASラインID
*           elid - 最後のADASラインID
*    output: nothing
*    return: 最寄りのADASレーンID (ないとき0)
************************************************************/
static int searchAdasLaneNearestToAdasLines(int slid, int elid)
{
  double cx=0, cy=0, cz=0;  // 重心位置計算用

  // 一連のADASラインデータの始点座標を積算して重心位置を求める
  for(int lid=slid; lid<=elid; lid++)
  {
    // ADASラインデータの検索
    MpAdLine_t::iterator itl = g_MpAdLine.find(lid);
    if(itl == g_MpAdLine.end())
    {
      std::cerr << "ERROR:[searchAdasLaneNearestToAdasLines-01] illegal lid=" << lid << std::endl;
      return 0;
    }

    // 始点のADASポイントデータの検索
    MpAdPoint_t::iterator itp = g_MpAdPoint.find(itl->second.BPID);
    if(itp == g_MpAdPoint.end())
    {
      std::cerr << "ERROR:[searchAdasLaneNearestToAdasLines-02] illegal lid=" << lid
                << ", pid=" << itl->second.BPID << std::endl;
      return 0;
    }
    // 始点重心位置の積算
    cx += itp->second.Bx;
    cy += itp->second.Ly;
    cz += itp->second.H;

    // 終点のADASポイントデータの検索
    itp = g_MpAdPoint.find(itl->second.FPID);
    if(itp == g_MpAdPoint.end())
    {
      std::cerr << "ERROR:[searchAdasLaneNearestToAdasLines-03] illegal lid=" << lid
                << ", pid=" << itl->second.FPID << std::endl;
      return 0;
    }
    // 終点重心位置の積算
    cx += itp->second.Bx;
    cy += itp->second.Ly;
    cz += itp->second.H;
  }

  // 車線リンクの中で最寄りのものを検索
  int pcnt = (elid - slid + 1) * 2;  // 積算ポイントデータの数
  int lkid = searchAdasLaneNearestToPoint(cx / pcnt, cy / pcnt, cz / pcnt);
  if(lkid == 0)
    std::cerr << "ERROR:[searchAdasLaneNearestToAdasLines-04] searchAdasLaneNearestToPoint()" << std::endl;

  return lkid;
}

/************************************************************
*  ADASエリアデータの、最寄りのADASレーンIDを調べる
*
*    input: aid - ADASエリアID
*    output: nothing
*    return: 最寄りのADASレーンID (ないとき0)
************************************************************/
static int searchAdasLaneNearestToAdasArea(int aid)
{
  // ADASエリアデータの検索
  MpAdArea_t::iterator it = g_MpAdArea.find(aid);
  if(it == g_MpAdArea.end())
  {
    std::cerr << "ERROR:[searchAdasLaneNearestToAdasArea-01] illegal aid=" << aid << std::endl;
    return 0;
  }

  // 車線リンクの中で最寄りのものを検索
  int lkid = searchAdasLaneNearestToAdasLines(it->second.SLID, it->second.ELID);
  if(lkid == 0)
    std::cerr << "ERROR:[searchAdasLaneNearestToAdasArea-02] searchAdasLaneNearestToAdasLines(),"
              << " aid=" << aid << std::endl;

  return lkid;
}

/************************************************************
*  個別データ(経度・緯度・標高・X座標・Y座標)からADASポイントデータ(B01)へのデータ変換
*
*    input: x - 経度 [deg]
*           y - 緯度 [deg]
*           z - 標高 [m]
*           bx - 平面直角座標系X座標 (北方向正) [m]
*           ly - 平面直角座標系Y座標 (東方向正) [m]
*           sn - 平面直角座標系 系番号 (1-19)
*    output: pid - ADASポイントID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranXyzToAdasPoint(double x, double y, double z, double bx, double ly, int sn, int& pid)
{
  if((sn < 1) || (sn > 19))
  {
    std::cerr << "ERROR:[tranXyzToAdasPoint-01] illegal sn=" << sn << std::endl;
    return false;
  }

  // 個別座標を、ADASポイントデータに変換
  insertAdasPoint(++c_PID, y, x, z, bx, ly, sn, 0,0,0);
  pid = c_PID;

  return true;
}

/************************************************************
*  ダイナミックマップのPointデータからADASポイントデータ(B01)へのデータ変換
*
*    input: dot - ダイナミックマップPointデータ
*    output: pid - ADASポイントID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpPointToAdasPoint(StDmpPoint_t pnt, int& pid)
{
  // Pointデータを、ADASポイントデータに変換
  return tranXyzToAdasPoint(pnt.xLon, pnt.yLat, pnt.z, pnt.y, pnt.x, pnt.sno, pid);
}

/************************************************************
*  ダイナミックマップのPointデータからADASベクトルデータ(B02)へのデータ変換
*
*    input: dot  - 起点のPointデータ
*           hang - 起点からの水平角 北0°時計回り [deg]
*           vang - 起点からの鉛直角 天頂0°下回り [deg]
*    output: vid - ADASベクトルID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpPointToAdasVector(StDmpPoint_t pnt, double hang, double vang, int& vid)
{
  int pid = 0;  // ADASポイントIDワーク

  // Pointデータを、ADASポイントデータに変換
  if(!tranDmpPointToAdasPoint(pnt, pid))
  {
    std::cerr << "ERROR:[tranDmpPointToAdasVector-01] tranDmpPointToAdasPoint()" << std::endl;
    return false;
  }

  // ADASベクトルデータ(B02)を格納
  insertAdasVector(++c_VID,  // ベクトルID
                   pid,    // 起点ポイントID
                   hang,   // 水平角
                   vang);  // 鉛直角

  vid = c_VID;
  return true;
}

/************************************************************
*  ダイナミックマップのPointデータからADASポールデータ(B03)へのデータ変換
*
*    input: pnt  - 起点のPointデータ
*           hang - 起点からの水平角 北0°時計回り [deg]
*           vang - 起点からの鉛直角 天頂0°下回り [deg]
*           length - 起点からの長さ [m]
*           dim  - 直径 [m]
*    output: plid - ADASポールID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpPointToAdasPole(StDmpPoint_t pnt, double hang, double vang,
                                   double length, double dim, int& plid)
{
  int vid = 0;  // ADASベクトルIDワーク

  // Pointデータを、ADASベクトルデータに変換
  if(!tranDmpPointToAdasVector(pnt, hang, vang, vid))
  {
    std::cerr << "WARNING:[tranDmpPointToAdasPole-01] tranDmpPointToAdasVector()" << std::endl;
    return false;
  }

  // ADASポールデータ(B03)を格納
  insertAdasPole(++c_PLID,  // ポールID
                vid,     // 起点と向きのベクトルID
                length,  // 起点からの長さ
                dim);    // 直径

  plid = c_PLID;
  return true;
}

/************************************************************
*  ダイナミックマップの2つのPointデータからADASラインデータ(B04)へのデータ変換
*
*    input: dot1 - 始点のPointデータ
*           dot2 - 終点のPointデータ
*    output: lid - ADASラインID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpPointsToAdasLine(StDmpPoint_t pnt1, StDmpPoint_t pnt2, int& lid)
{
  int pid1=0, pid2=0;  // ADASポイントIDワーク

  if(!tranDmpPointToAdasPoint(pnt1, pid1))
  {
    std::cerr << "WARNING:[tranDmpPointsToAdasLine-01] tranDmpPointToAdasPoint()" << std::endl;
    return false;
  }

  if(!tranDmpPointToAdasPoint(pnt2, pid2))
  {
    std::cerr << "WARNING:[tranDmpPointsToAdasLine-02] tranDmpPointToAdasPoint()" << std::endl;
    return false;
  }

  // ADASラインデータ(B04)を格納
  insertAdasLine(++c_LID,  // ラインのID
                 pid1,  // 始点ポイントID (線分の1点目)
                 pid2,  // 終点ポイントID (線分の2点目)
                 0,     // 前のラインID (0:この線分が最初の線分)
                 0);    // 次のラインID (0:この線分が終端の線分)

  lid = c_LID;
  return true;
}

#if 0
/************************************************************
*  ダイナミックマップのLineデータからADASラインデータ(B04)へのデータ変換
*
*    input: pnts - Lineデータ (ライン状のPointデータリスト)
*    output: blid - 最初のADASラインID
*            flid - 最後のADASラインID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpLineToAdasLines(VcDmpPoint_t pnts, int& blid, int& flid)
{
  int bPID;     // 開始ADASポイントID
  int tBLID;    // 前のADASラインID
  int tFLID;    // 次のADASラインID
  int pid = 0;  // ADASポイントIDワーク

  if(pnts.size() < 2)
  {
    std::cerr << "WARNING:[tranDmpLineToAdasLines-01] illegal line size" << std::endl;
    return false;
  }

  bPID = c_PID + 1;  // 開始ADASポイントID保存
  blid = c_LID + 1;  // 最初のADASラインIDの保存

  // Pointデータリストの一連の座標を、ADASポイントデータに変換
  for(VcDmpPoint_t::const_iterator itp = pnts.begin(); itp != pnts.end(); ++itp)
  {
    if(!tranDmpPointToAdasPoint(*itp, pid))
    {
      std::cerr << "WARNING:[tranDmpLineToAdasLines-02] tranDmpPointToAdasPoint()" << std::endl;
      return false;
    }
  }

  int lcnt = c_PID - bPID;  // 生成するADASラインデータの数

  // ADASポイントデータから、ADASラインデータに変換
  for(int i=0; i < lcnt; i++)
  {
    if(i == 0)
      tBLID = 0;  // 最初の線分
    else
      tBLID = c_LID;

    if(i == (lcnt - 1))
      tFLID = 0;  // 最後の線分
    else
      tFLID = c_LID + 2;

    // ADASラインデータ(B04)を格納
    insertAdasLine(++c_LID,  // ラインのID
                   i + bPID,  // 始点ポイントID (線分の1点目)
                   i + bPID + 1, // 終点ポイントID (線分の2点目)
                   tBLID,    // 前のラインID (0:この線分が最初の線分)
                   tFLID);   // 次のラインID (0:この線分が終端の線分)
  }

  flid = c_LID;
  return true;
}
#endif

/************************************************************
*  ダイナミックマップのAreaデータからADASエリアデータ(B05)へのデータ変換
*
*    input: pnts　- Areaデータ (ポリゴン状のPointデータリスト)
*    output: aid - ADASエリアID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpAreaToAdasArea(VcDmpPoint_t pnts, int& aid)
{
  int bPID;   // 開始ADASポイントID
  int sLID;   // 最初のADASラインID
  int tBLID;  // 前のADASラインID
  int tFLID;  // 次のADASラインID

  if(pnts.size() < 2)
  {
    std::cerr << "WARNING:[tranDmpAreaToAdasArea-01] illegal points size" << std::endl;
    return false;
  }

  bPID = c_PID + 1;  // 開始ADASポイントID保存
  sLID = c_LID + 1;  // 最初のラインIDの保存

  // XYZリストの一連の座標を、ADASポイントデータに変換
  int pid = 0;
  for(VcDmpPoint_t::const_iterator itp = pnts.begin(); itp != pnts.end(); ++itp)
  {
    if(!tranDmpPointToAdasPoint(*itp, pid))
    {
      std::cerr << "WARNING:[tranDmpAreaToAdasArea-02] tranDmpAreaToAdasPoint()" << std::endl;
      return false;
    }
  }

  int lcnt = c_PID - bPID;  // 生成するADASラインデータの数

  // ADASポイントデータから、ADASラインデータに変換
  for(int i=0; i < lcnt; i++)
  {
    if(i == 0)
      tBLID = 0;  // 最初の線分
    else
      tBLID = c_LID;

    if(i == (lcnt - 1))
      tFLID = 0;  // 最後の線分
    else
      tFLID = c_LID + 2;

    // ADASラインデータ(B04)を格納
    insertAdasLine(++c_LID,  // ラインのID
                   i + bPID,  // 始点ポイントID (線分の1点目)
                   i + bPID + 1, // 終点ポイントID (線分の2点目)
                   tBLID,    // 前のラインID (0:この線分が最初の線分)
                   tFLID);   // 次のラインID (0:この線分が終端の線分)
  }

  // ADASエリアデータ(B05)を格納
  insertAdasArea(++c_AID, sLID, c_LID);

  aid = c_AID;
  return true;
}

/************************************************************
*  ダイナミックマップのポイントリストからADASノードデータ(N02)へのデータ変換
*
*    input: pnts - Pointデータリスト (ポイント状)
*    output: bnid - 最初のノードID
*            fnid - 最後のノードID
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool tranDmpPointsToAdasNode(VcDmpPoint_t pnts, int& bnid, int& fnid)
{
  bnid = c_NID + 1;  // 最初のADASノードIDを保存

  // ポイントリストの一連の座標を、ADASポイントデータに変換
  for(VcDmpPoint_t::const_iterator itp = pnts.begin(); itp != pnts.end(); ++itp)
  {
    int pid = 0;
    if(!tranDmpPointToAdasPoint(*itp, pid))
    {
      std::cerr << "WARNING:[tranDmpPointsToAdasNode-01] tranDmpPointToAdasPoint()" << std::endl;
      return false;
    }

    // ADASノードデータ(N02)を格納
    insertAdasNode(++c_NID,  // ノードID
                   pid);  // ポイントクラスID
  }

  fnid = c_NID;  // 最後のADASノードIDを保存

  return true;
}

/************************************************************
*  [FC8110]レーンのデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADASレーンデータ(N03)、ADAS中心線形データ(N01)、 ADAS走行エリア(N04)へ変換
*          ADASレーンデータへの変換のみ行い、繋ぎ込みは別途行う
************************************************************/
static bool tranDmpLaneToAdasLane(void)
{
  StLaneExBuff_t lnExBuff;      // 拡張レーンバッファ書き込み用
  StLaneBgnEndBuff_t beExBuff;  // 始点終点レーンバッファ書き込み用
  int bnid;     // 最初のノードID
  int fnid;     // 最後のノードID
  int tBLID;    // 手前のADASレーンID
  int tFLID;    // 次のADASレーンID
  int tBNID;    // 始点ノードID
  int lncnt;    // 一つのDmpレーン内の生成するADASレーンデータ数
  double dist;  // 一つのDmpレーン内の起点からの追加距離(起点0)
  double x1, y1, z1, x2, y2, z2;

  // Dmpレーン毎のデータ変換
  for(MpDmpLane_t::const_iterator it = g_MpDmpLane.begin(); it != g_MpDmpLane.end(); ++it)
  {
    // [FA8120]レーン走行可能領域のデータ変換
    int waid = 0;  // 走行エリアID
    if(it->second.sf2_mbr.size() > 3)
    {
      // 一連の座標データから、ADASエリアデータに変換
      int aid = 0;
      if(!tranDmpAreaToAdasArea(it->second.sf2_mbr, aid))
      {
        std::cerr << "ERROR:[tranDmpLaneToAdasLane-01] tranDmpAreaToAdasArea()" << std::endl;
        return false;
      }

      // 走行エリア(N04)のデータ登録
      insertAdasWayarea(++c_WAID,  // 走行エリアID
                        aid);      // エリアクラスID
      waid = c_WAID;

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8110]Lane feature_id=" << it->second.feature_id
               << " -> (N04)WAID=" << c_WAID << std::endl;
    }

    // [FL8130]レーン中心線/走行目安線のデータ変換
    if(it->second.se1_geometry.size() < 2)
    {
      std::cerr << "WARNING:[tranDmpLaneToAdasLane-01] illegal geometry size, "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    // 一連の座標をADASノードデータに変換
    if(!tranDmpPointsToAdasNode(it->second.se1_geometry, bnid, fnid))
    {
      std::cerr << "WARNING:[tranDmpLaneToAdasLane-02] tranDmDotsToAdasNode(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    dist = 0;  // 起点からの追加距離をクリア(起点0)
    lncnt = fnid - bnid;  // 生成するADASレーンデータ数

    // 一連のADASノードデータから、ADASレーンデータに変換
    for(int i=0; i < lncnt; i++)
    {
      if(i == 0)
        tBLID = 0;  // 始点データ
      else
        tBLID = c_LnID;

      if(i == (lncnt - 1))
        tFLID = 0;  // 終点データ
      else
        tFLID = c_LnID + 2;

      // ADAS始点ノードの座標を取得
      tBNID = i + bnid;  // 現在処理中の始点ノードID
      if(!getAdasNodePoint(tBNID, x1, y1, z1))
      {
        std::cerr << "ERROR:[tranDmpLaneToAdasLane-03] illegal nid=" << i + bnid << std::endl;
        return false;
      }

      // ADAS終点ノードの座標を取得
      if(!getAdasNodePoint(tBNID + 1, x2, y2, z2))
      {
        std::cerr << "ERROR:[tranDmpLaneToAdasLane-04] illegal nid=" << i + bnid + 1 << std::endl;
        return false;
      }

      double span = hypot(x2 - x1, y2 - y1);  // 区間距離
      double dir = atan2(x2 - x1, y2 - y1);  // 方向角 [rad]
      if(dir < 0)
        dir += M_PI * 2;

      // レーンデータ(N03)を格納
      insertAdasLane(++c_LnID, // レーンID
                     ++c_DID,  // 中心線形ID
                     tBLID,    // 手前のレーンID (始点データは0)
                     tFLID,    // 次のレーンID (終点データは0)
                     tBNID,    // 始点ノードID (この線分の始点ノードID)
                     tBNID + 1, // 終点ノードID (この線分の終点ノードID)
                     0,        // 分岐合流パターン 通常区間(0)、左(1)右(2)へ分岐、左(3)右(4)から合流
                     0, 0, 0,  // 合流ID2, ID3, ID4
                     0, 0, 0,  // 分岐ID2, ID3, ID4
                     0,        // 交差点ID (このレコードが交差点に含まれる場合、そのID。通常は0)
                     span,     // 区間距離 (この線分の長さ)
                     it->second.iA_NL,  // 車線数 (交差点内は0)
                     1,        // レーン番号 (左から1、交差点内は0)
                     0,        // レーン種別 (0:直進レーン、1:左折レーン、2:右折レーン。料金所レーン等もここで区別させる)
                     it->second.iA_SP,  // 法定速度
                     it->second.iA_SP,  // 目標速度 (走行時の想定速度。現在は、法定速度以内の最大速度を設定)
                     0,        // 道路区間ID (交差点内は0。隣り合うレーンは、通常同じ区間IDを持つ)
                     0,        // 車線変更可/不可 (0:可、1:不可、交差点内は0)
                     waid);    // 属する走行エリアID

      int pid = 0;  // ADASポイントID
      tranDmpPointToAdasPoint(it->second.se1_geometry[i], pid);  // ADASポイントデータ登録
      double apara = 0;    // クロソイドパラメータ (単曲線、直線は0)
      double r = 9.0e+10;  // 回転半径 (時計回りマイナス、直線区間、クロソイド起点は9.0E+10)

      // 次にADAS中心線形データ登録
      insertAdasDtlene(c_DID,  // レコードID
                       dist,   // 追加距離
                       pid,    // 位置ポイントID 軌道ポイント位置
                       dir,    // 方向角 [ラジアン単位]
                       apara,  // クロソイドパラメータ (単曲線、直線は0)
                       r,      // 回転半径 (時計回りマイナス、直線区間、クロソイド起点は9.0E+10)
                       it->second.dA_PN,  // 縦断勾配 [%表現]
                       0,      // 横断勾配 [%表現]
                       it->second.dA_WL,   // 左幅員
                       it->second.dA_WR);  // 右幅員
      dist += span;

      // レーンID情報を車線リンク拡張レーンバッファへ格納
      lnExBuff.feature_id = it->second.feature_id;  // Dmpレーン地物ID
      lnExBuff.sno = i;           // Dmpレーン格納順
      lnExBuff.ad_LnID = c_LnID;  // ADASレーンID
      lnExBuff.xRct = x1;         // 平面直角系X座標 [m]
      lnExBuff.yRct = y1;         // 平面直角系Y座標 [m]
      lnExBuff.zHgt = z1;         // 標高 [m]
      mVcLaneExBuff.push_back(lnExBuff);

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8110]Lane feature_id=" << it->second.feature_id << ", no=" << i
               << " -> (N03)LaneID=" << c_LnID << ", (N01)DtlaneID=" << c_DID << std::endl;

      if(i == 0)
      {
        // 始点車線リンクIDとADASレーンIDを、車線リンク始点終点レーンバッファの始点へ格納
        beExBuff.bgnLnID = c_LnID;
      }
    }

    // 終点車線リンクIDとADASレーンIDを、車線リンク始点終点レーンバッファへ格納
    beExBuff.feature_id = it->second.feature_id;
    beExBuff.endLnID = c_LnID;
    mMpLaneBgnEndBuff.insert(std::make_pair(beExBuff.feature_id, beExBuff));
  }

  return true;
}

#if 0
/************************************************************
*  [FC8140]歩道のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS歩道(S002)へ変換
************************************************************/
static bool tranDmpExtendedPedestrianToAdasSidewalk(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpExtendedPedestrian_t::const_iterator it = g_MpDmpExtendedPedestrian.begin();
      it != g_MpDmpExtendedPedestrian.end(); ++it)
  {
    for(VcDmpPedestrianArea_t::const_iterator ita = it->second.mVcDmpPedestrianArea.begin();
        ita != it->second.mVcDmpPedestrianArea.end(); ++ita)
    {
      // 一連の座標データから、ADAS歩道データに変換
      int aid = 0;
      if(!tranDmpAreaToAdasArea(ita->sf_mbr, aid))
      {
        std::cerr << "ERROR:[tranDmpExtendedPedestrianToAdasAdasSidewalk-03] tranDmpAreaToAdasArea(), "
                  << "feature_id=" << ita->feature_id << std::endl;
        return false;
      }

      // 最寄りのADASレーンIDを検索
      int lkid = searchAdasLaneNearestToAdasArea(aid);
      if(lkid == 0)
        std::cerr << "WARNING:[tranDmpExtendedPedestrianToAdasSidewalk-04] searchAdasLaneNearestToAdasLines(), "
                  << "feature_id=" << ita->feature_id << std::endl;

      // ADAS歩道(S002)を格納
      insertAdasSidewalk(++c_SidewalkID, // 歩道ID
                         aid,    // エリアクラスID
                         lkid);  // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8140]ExtendedPedestrian=" << it->second.feature_id
               << " -> (P004)SidewalkID=" << c_SidewalkID << std::endl;
    }
  }

  return true;
}
#endif

/************************************************************
*  [FC8145]横断歩道のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS横断歩道(P004)へ変換
************************************************************/
static bool tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpExtendedPedestrianCrossing_t::const_iterator it = g_MpDmpExtendedPedestrianCrossing.begin();
      it != g_MpDmpExtendedPedestrianCrossing.end(); ++it)
  {
    // 個別の歩道領域毎のデータ変換
    for(VcDmpPedestrianArea_t::const_iterator ita = it->second.mVcDmpPedestrianArea.begin();
        ita != it->second.mVcDmpPedestrianArea.end(); ++ita)
    {
      // 一連の座標データから、ADAS横断歩道データに変換
      int aid = 0;
      if(!tranDmpAreaToAdasArea(ita->sf_mbr, aid))
      {
        std::cerr << "ERROR:[tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk-03] tranDmpAreaToAdasArea(), "
                  << "feature_id=" << ita->feature_id << std::endl;
        return false;
      }

      // 最寄りのADASレーンIDを検索
      int lkid = searchAdasLaneNearestToAdasArea(aid);
      if(lkid == 0)
        std::cerr << "WARNING:[tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk-04] searchAdasLaneNearestToAdasLines(), "
                  << "feature_id=" << ita->feature_id << std::endl;

      // ADAS横断歩道(P004)を格納
      insertAdasCrosswalk(++c_CrosswalkID,  // 横断歩道ID
                          aid,  // エリアクラスID
                          0,    // 種別 (0:外枠、1:縞模様、2:自転車通行帯)
                          0,    // 外枠ID
                          lkid);  // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8145]ExtendedPedestrianCrossing=" << it->second.feature_id
               << " -> (P004)CrosswalkID=" << c_CrosswalkID << std::endl;
    }

    // 個別の横断歩道模様毎のデータ変換
    for(VcDmpPedestrianCrossingMarkingsShape_t::const_iterator ita = it->second.mVcDmpPedestrianCrossingMarkingsShape.begin();
        ita != it->second.mVcDmpPedestrianCrossingMarkingsShape.end(); ++ita)
    {
      // 一連の座標データから、ADAS横断歩道データに変換
      int aid = 0;
      if(!tranDmpAreaToAdasArea(ita->sf_mbr, aid))
      {
        std::cerr << "ERROR:[tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk-03] tranDmpAreaToAdasArea(), "
                  << "feature_id=" << ita->feature_id << std::endl;
        return false;
      }

      // 最寄りのADASレーンIDを検索
      int lkid = searchAdasLaneNearestToAdasArea(aid);
      if(lkid == 0)
        std::cerr << "WARNING:[tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk-04] searchAdasLaneNearestToAdasLines(), "
                  << "feature_id=" << ita->feature_id << std::endl;

      // ADAS横断歩道(P004)を格納
      insertAdasCrosswalk(++c_CrosswalkID,  // 横断歩道ID
                          aid,  // エリアクラスID
                          1,    // 種別 (0:外枠、1:縞模様、2:自転車通行帯)
                          0,    // 外枠ID
                          lkid);  // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8145]ExtendedPedestrianCrossing=" << it->second.feature_id
               << " -> (P004)CrosswalkID=" << c_CrosswalkID << std::endl;
    }
  }

  return true;
}

/************************************************************
*  [FA8170]交差点領域のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS交差点(R004)へ変換
************************************************************/
static bool tranDmpIntersectionAreaShapeToAdasIntersection(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpIntersectionAreaShape_t::const_iterator it = g_MpDmpIntersectionAreaShape.begin();
      it != g_MpDmpIntersectionAreaShape.end(); ++it)
  {
    // 一連の座標データから、ADASエリアデータに変換
    int aid = 0;
    if(!tranDmpAreaToAdasArea(it->second.sf_mbr, aid))
    {
      std::cerr << "ERROR:[tranDmpIntersectionAreaShapeToAdasIntersection-01] tranDmcDotsToAdasArea(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToAdasArea(aid);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpIntersectionAreaShapeToAdasIntersection-02] searchAdasLaneNearestToAdasArea(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADAS交差点(R004)を格納
    insertAdasIntersection(++c_IntersectionID,  // 交差点ID
                           aid,    // エリアクラスID
                           lkid);  // 最寄りのレーンID

    // 交差点情報を交差点領域拡張バッファへ格納
    StIntersectionAreaExBuff_t exBuff;
    StXyPoint_t pBuff;
    double xmin = 99999999.0, xmax = -99999999.0;
    double ymin = 99999999.0, ymax = -99999999.0;
    for(std::vector<StDmpPoint_t>::const_iterator itd = it->second.sf_mbr.begin();
        itd != it->second.sf_mbr.end(); itd++)
    {
      pBuff.xRct = itd->x;
      pBuff.yRct = itd->y;
      if(xmin > pBuff.xRct)
        xmin = pBuff.xRct;
      else if(xmax < pBuff.xRct)
        xmax = pBuff.xRct;
      if(ymin > pBuff.yRct)
        ymin = pBuff.yRct;
      else if(ymax < pBuff.yRct)
        ymax = pBuff.yRct;
      exBuff.pgn.push_back(pBuff);
    }
    exBuff.IsID = c_IntersectionID;  // 対応するADAS交差点ID
    exBuff.xmin = xmin;
    exBuff.xmax = xmax;
    exBuff.ymin = ymin;
    exBuff.ymax = ymax;
    mVcIntersectionAreaExBuff.push_back(exBuff);

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FA8170]IntersectionAreaShape feature_id=" << it->second.feature_id
             << " -> (R004)IntersectionID=" << c_IntersectionID << std::endl;
  }

  return true;
}

/************************************************************
*  [FP8210]ポールのデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: [FP8210]ポール->ADASポール(K001)へ変換
************************************************************/
static bool tranDmpPoleToAdasPole(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpPole_t::const_iterator it = g_MpDmpPole.begin(); it != g_MpDmpPole.end(); ++it)
  {
    // ADASポールデータに変換
    int plid = 0;
    if(!tranDmpPointToAdasPole(it->second.sn_geometry,  // 起点のPointデータ
                               it->second.dA_YN,  // 起点からの水平角
                               it->second.dA_PN,  // 起点からの鉛直角
                               it->second.dA_LM,  // 起点からの長さ
                               it->second.dA_DA,  // 直径
                               plid))
    {
      std::cerr << "ERROR:[tranDmpPoleToAdasPole-01] tranDmcPointToAdasPole(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      continue;
    }

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToPoint(it->second.sn_geometry.x, it->second.sn_geometry.y, it->second.sn_geometry.z);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpPoleToAdasPole-02] searchAdasLaneNearestToPoint(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADASポール(K001)を格納
    insertAdasPoledata(++c_PoledataID,  // ポールID
                       plid,  // ポールクラスID
                       lkid);  // 最寄りのレーンID

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FP8210]Pole feature_id=" << it->second.feature_id
             << " -> (K001)PoledataID=" << c_PoledataID << std::endl;
  }

  return true;
}

/************************************************************
*  [FC8220]標識のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS標識(K003)へ変換
************************************************************/
static bool tranDmpExtendedTrafficSignToAdasRoadsign(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpExtendedTrafficSign_t::const_iterator it = g_MpDmpExtendedTrafficSign.begin();
      it != g_MpDmpExtendedTrafficSign.end(); ++it)
  {
    // 個々のポール毎のデータ変換
    for(VcDmpPole_t::const_iterator itp = it->second.mVcDmpPole.begin();
        itp != it->second.mVcDmpPole.end(); ++itp)
    {
      // ポールを求める
      int plid = 0;
      if(!tranDmpPointToAdasPole(itp->sn_geometry,  // 起点のPointデータ
                                 itp->dA_YN,  // 起点からの水平角
                                 itp->dA_PN,  // 起点からの鉛直角
                                 itp->dA_LM,  // 起点からの長さ
                                 itp->dA_DA,  // 直径
                                 plid))
      {
        std::cerr << "ERROR:[tranDmcExtendedTrafficSignToAdasRoadsign-01] tranDmpPointToAdasPole(), "
                  << "feature_id=" << itp->feature_id << std::endl;
        continue;
      }

      // ADASポール(K001)を格納
      insertAdasPoledata(++c_PoledataID,  // ポールID
                         plid,  // ポールクラスID
                         0);    // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8220]ExtendedTrafficSign feature_id=" << it->second.feature_id
               << " -> (K001)PoledataID=" << c_PoledataID << std::endl;
    }

    // 標識を求める
    int vid = 0;
    if(!tranDmpPointToAdasVector(it->second.sn2_geometry, it->second.dA_YN, it->second.dA_PN, vid))
    {
      std::cerr << "ERROR:[tranDmcExtendedTrafficSignToAdasRoadsign-02] tranDmpPointToAdasVector(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      continue;
    }

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToPoint(it->second.sn2_geometry.x, it->second.sn2_geometry.y, it->second.sn2_geometry.z);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmcExtendedTrafficSignToAdasRoadsign-03] searchAdasLaneNearestToPoint(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADAS標識(K003)を格納
    insertAdasRoadsign(++c_RoadsignID,  // 標識ID
                       vid,  // ベクトルクラスID (標識の中心から標識の向いている向きへベクトル)
                       0,    // ポールクラスID (この標識のポール、0も可)
                       "1",
                       lkid);  // 最寄りのレーンID

    // 標識情報を標識拡張バッファへ格納
    StRoadsignExBuff_t stExBuff;
    stExBuff.feature_id = it->first;  // Dmp標識地物ID
    stExBuff.ad_RoadsignID = c_RoadsignID;  // 対応するADAS標識ID
    stExBuff.xRct = it->second.sn2_geometry.x;
    stExBuff.yRct = it->second.sn2_geometry.y;
    stExBuff.zHgt = it->second.sn2_geometry.z;
    std::pair<MpRoadsignExBuff_t::iterator, bool> result =
      mMpRoadsignExBuff.insert(std::make_pair(stExBuff.feature_id, stExBuff));
    if(!result.second)
    {
      std::cerr << "WARNING:[tranDmcExtendedTrafficSignToAdasRoadsign-04] mMpRoadsignExBuff.insert()" << std::endl;
      return false;
    }

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FC8220]ExtendedTrafficSign feature_id=" << it->second.feature_id
             << " -> (K003)RoadsignID=" << c_RoadsignID << std::endl;
  }

  return true;
}

/************************************************************
*  [FC8230]信号のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS信号(K004)へ変換
************************************************************/
static bool tranDmpExtendedTrafficLightToAdasSignaldata(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpExtendedTrafficLight_t::const_iterator it = g_MpDmpExtendedTrafficLight.begin();
      it != g_MpDmpExtendedTrafficLight.end(); ++it)
  {
    // 信号拡張バッファの準備
      StSignaldataExBuff_t stExBuff;
      stExBuff.feature_id = it->second.feature_id;  // Dmp信号地物ID
      stExBuff.ad_SignalLamps.clear();

    // 個々のポール毎のデータ変換
    for(VcDmpPole_t::const_iterator itp = it->second.mVcDmpPole.begin();
        itp != it->second.mVcDmpPole.end(); ++itp)
    {
      // ポールを求める
      int plid = 0;
      if(!tranDmpPointToAdasPole(itp->sn_geometry,  // 起点のPointデータ
                                 itp->dA_YN,  // 起点からの水平角
                                 itp->dA_PN,  // 起点からの鉛直角
                                 itp->dA_LM,  // 起点からの長さ
                                 itp->dA_DA,  // 直径
                                 plid))
      {
        std::cerr << "ERROR:[tranDmpExtendedTrafficLightToAdasSignaldata-01] tranDmpPointToAdasPole(), "
                  << "feature_id=" << itp->feature_id << std::endl;
        continue;
      }

      // ADASポール(K001)を格納
      insertAdasPoledata(++c_PoledataID,  // ポールID
                         plid,  // ポールクラスID
                         0);  // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8230]ExtendedTrafficLight Pole feature_id=" << it->second.feature_id
               << " -> (K001)PoledataID=" << c_PoledataID << std::endl;
    }

    // 個々の信号ランプ毎のデータ変換
    for(VcDmpTrafficLightLamp_t::const_iterator itl = it->second.mVcTrafficLightLamp.begin();
        itl != it->second.mVcTrafficLightLamp.end(); ++itl)
    {
      // 信号ランプのベクトルを求める
      int vid = 0;
      if(!tranDmpPointToAdasVector(itl->sn_geometry, it->second.dA_YN, it->second.dA_PN, vid))
      {
        std::cerr << "ERROR:[tranDmpExtendedTrafficLightToAdasSignaldata-02] tranAdasLaneToFaceAdasVector(), "
                  << "feature_id=" << itl->feature_id << std::endl;
        continue;
      }

      // ランプ種別の処理
      int iType;
      if(((it->second.iA_LI == 1) || (it->second.iA_LI == 3)) && (itl->iA_LY == 1))
        iType = 1;  // 車両、赤
      else if(((it->second.iA_LI == 1) || (it->second.iA_LI == 3)) && (itl->iA_LY == 2))
        iType = 2;  // 車両、青
      else if(((it->second.iA_LI == 1) || (it->second.iA_LI == 3)) && (itl->iA_LY == 3))
        iType = 3;  // 車両、黄
      else if((it->second.iA_LI == 2) && (itl->iA_LY == 1))
        iType = 4;  // 歩行者、赤
      else if((it->second.iA_LI == 2) && (itl->iA_LY == 2))
        iType = 5;  // 歩行者、青
      else
        iType = 9;  // その他

      // ADAS信号(K004)を格納
      insertAdasSignaldata(++c_SignaldataID,  // 信号ID
                           vid,  // ベクトルクラスID (ランプの中心からランプの向いている方向へベクトル)
                           0,    // ポールクラスID (この信号ランプのポール、0も可)
                           iType,  // 種別 (赤(1),青(2),黄(3),歩行者赤(4),歩行者青(5),その他(9))
                           0);     // 最寄りのレーンID

      // 各信号ランプ情報を信号拡張バッファへ格納
      StSignalLampExBuff_t stLampExBuff;
      stLampExBuff.ad_SignaldataID = c_SignaldataID;  // 対応するADAS信号ID
      stLampExBuff.xRct = itl->sn_geometry.x;
      stLampExBuff.yRct = itl->sn_geometry.y;
      stLampExBuff.zHgt = itl->sn_geometry.z;
      stExBuff.ad_SignalLamps.push_back(stLampExBuff);

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8230]ExtendedTrafficLight feature_id=" << it->second.feature_id
               << ", TrafficLightType=" << itl->iA_LY
               << " -> (K004)SignaldataID=" << c_SignaldataID << std::endl;
    }

    // 信号情報を信号拡張バッファへ格納
    std::pair<MpSignaldataExBuff_t::iterator, bool> result =
      mMpSignaldataExBuff.insert(std::make_pair(stExBuff.feature_id, stExBuff));
    if(!result.second)
    {
      std::cerr << "WARNING:[tranDmpExtendedTrafficLightToAdasSignaldata-03] mMpSignaldataExBuff.insert()" << std::endl;
      return false;
    }
  }

  return true;
}

#if 0
/************************************************************
*  [FC8240]街灯のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS街灯(K005)へ変換
************************************************************/
static bool tranDmpExtendedLightingToAdasStreetlight(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpExtendedLighting_t::const_iterator it = g_MpDmpExtendedLighting.begin(); it != g_MpDmpExtendedLighting.end(); ++it)
  {
    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToPoint(it->second.sn1_geometry.x, it->second.sn1_geometry.y, it->second.sn1_geometry.z);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpExtendedLightingToAdasStreetlight-01] searchAdasLaneNearestToPoint(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADASポールデータに変換
    int plid = 0;
    if(!tranDmpPointToAdasPole(it->second.sn1_geometry,  // 起点のPointデータ
                               it->second.dA_YN,  // 起点からの水平角
                               it->second.dA_PN,  // 起点からの鉛直角
                               it->second.dA_LM,  // 起点からの長さ
                               it->second.dA_DA,  // 直径
                               plid))
    {
      std::cerr << "ERROR:[tranDmpExtendedLightingToAdasStreetlight-02] tranDmpPointToAdasPole(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      continue;
    }

    //  ADASラインデータに変換
    int lid = 0;
    if(!tranDmpPointsToAdasLine(it->second.se2_geometry[0], it->second.se2_geometry[1], lid))
    {
      std::cerr << "ERROR:[tranDmpExtendedLightingToAdasStreetlight-03] tranDmcDotToAdasLine(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      continue;
    }

    // ADAS街灯(K005)を格納
    insertAdasStreetlight(++c_StreetlightID,  // 街灯ID
                          lid,   // ラインクラスID (街灯ランプをラインで取る)
                          plid,  // ポールクラスID
                          lkid);  // 最寄りのレーンID

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FC8240]ExtendedLighting feature_id=" << it->second.feature_id
             << " -> (K005)StreetlightID=" << c_StreetlightID << std::endl;
  }

  return true;
}
#endif

#if 0
/************************************************************
*  [FC8250]路面マークのデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS路面マーク(P005)へ変換
************************************************************/
static bool tranDmpExtendedRoadMarkingsToAdasRoadmark(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpExtendedRoadMarkings_t::const_iterator it = g_MpDmpExtendedRoadMarkings.begin();
      it != g_MpDmpExtendedRoadMarkings.end(); ++it)
  {
    // 個別の路面マーク毎のデータ変換
    for(VcDmpRoadMarkingsShape_t::const_iterator ita = it->second.mVcDmpRoadMarkingsShape.begin();
        ita != it->second.mVcDmpRoadMarkingsShape.end(); ++ita)
    {
      // 一連の座標データから、ADASエリアデータに変換
      int aid = 0;
      if(!tranDmpAreaToAdasArea(ita->sf_mbr, aid))
      {
        std::cerr << "ERROR:[tranDmpExtendedRoadMarkingsToAdasRoadmark-01] tranDmpAreaToAdasArea(), "
                  << "feature_id=" << ita->feature_id << std::endl;
        return false;
      }

      // 最寄りのADASレーンIDを検索
      int lkid = searchAdasLaneNearestToAdasArea(aid);
      if(lkid == 0)
        std::cerr << "WARNING:[tranDmpExtendedRoadMarkingsToAdasRoadmark-02] searchAdasLaneNearestToAdasLines(), "
                  << "feature_id=" << ita->feature_id << std::endl;

      // ADAS路面マーク(P005)を格納
      insertAdasRoadmark(++c_RoadmarkID, // 路面マークID
                         aid,    // エリアクラスID
                         "1",    // 種別
                         lkid);  // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "[FC8250]ExtendedRoadMarkings=" << it->second.feature_id
               << " -> (P004)RoadmarkID=" << c_RoadmarkID << std::endl;
    }
  }

  return true;
}
#endif

/************************************************************
*  [FL8260]停止線のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS路面マーク(P005)へ変換
************************************************************/
static bool tranDmpStopLineToAdasStopline(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpStopLine_t::const_iterator it = g_MpDmpStopLine.begin();
      it != g_MpDmpStopLine.end(); ++it)
  {
    // 2つの座標データから、一本のADASラインデータに変換
    if(it->second.se_geometry.size() != 2)
    {
      std::cerr << "ERROR:[tranDmpStopLineToAdasStopline-01] illegal geometry size, "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    int lid = 0;
    if(!tranDmpPointsToAdasLine(it->second.se_geometry[0], it->second.se_geometry[1], lid))
    {
      std::cerr << "ERROR:[tranDmpStopLineToAdasStopline-02] tranDmpPointsToAdasLine(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToAdasLines(lid, lid);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpStopLineToAdasStopline-03] searchAdasLaneNearestToAdasLines(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADAS停止線(P002)を格納
    insertAdasStopline(++c_StoplineID, // 停止線ID
                       lid,    // ラインクラスID
                       0,      // 信号ランプID (関連する信号ランプのID(赤信号を想定)、0も可)
                       0,      // 標識ID (関連する標識のID(止まれ標識を想定)、0も可)
                       lkid);  // 最寄りのレーンID

    // 停止線情報を停止線拡張バッファへ格納
    StStoplineExBuff_t stExBuff;
    stExBuff.feature_id = it->first;        // Dmp停止線地物ID
    stExBuff.ad_StoplineID = c_StoplineID;  // 対応するADAS停止線ID
    stExBuff.xRct = (it->second.se_geometry[0].x + it->second.se_geometry[1].x) / 2;
    stExBuff.yRct = (it->second.se_geometry[0].y + it->second.se_geometry[1].y) / 2;
    stExBuff.zHgt = (it->second.se_geometry[0].z + it->second.se_geometry[1].z) / 2;
    std::pair<MpStoplineExBuff_t::iterator, bool> result =
      mMpStoplineExBuff.insert(std::make_pair(stExBuff.feature_id, stExBuff));
    if(!result.second)
    {
      std::cerr << "WARNING:[tranDmpStopLineToAdasStopline-04] mMpStoplineExBuff.insert()" << std::endl;
      return false;
    }

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FL8260]StopLine=" << it->second.feature_id
             << " -> (P002)StoplineID=" << c_StoplineID << std::endl;
  }

  return true;
}

#if 0
/************************************************************
*  [FL8310]道路縁のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS道路縁(R001)へ変換
************************************************************/
static bool tranDmpRoadEdgeToAdasRoadedge(void)
{
  int slid;  // 最初のラインID
  int flid;  // 最後のラインID
  int lcnt;  // 道路縁データのライン数

  // 個別地物毎のデータ変換
  for(MpDmpRoadEdge_t::const_iterator it = g_MpDmpRoadEdge.begin();
      it != g_MpDmpRoadEdge.end(); ++it)
  {
    // 一連の座標データから、ADASラインデータに変換
    if(!tranDmpLineToAdasLines(it->second.se_geometry,  slid, flid))
    {
      std::cerr << "ERROR:[tranDmpRoadEdgeToAdasRoadedge-01] tranDmpLineToAdasLines(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    lcnt = flid - slid;  // 生成する道路縁データの数

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToAdasLines(slid, flid);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpRoadEdgeToAdasRoadedge-02] searchAdasLaneNearestToAdasLine(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADAS道路縁(R001)を格納
    for(int i=0; i<lcnt; i++)
    {
      insertAdasRoadedge(++c_RoadedgeID,  // 道路縁ID
                         slid + i,  // ラインクラスID
                         0);        // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
      ofsdbg << "[FL8310]RoadEdge=" << it->second.feature_id << ", i=" << i
            << " -> (R001)RoadedgeID=" << c_RoadedgeID << std::endl;
    }
  }

  return true;
}
#endif

#if 0
/************************************************************
*  [FL8311]縁石のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS縁石(R003)へ変換
************************************************************/
static bool tranDmpCurbToAdasCurb(void)
{
  int slid;  // 最初のラインID
  int flid;  // 最後のラインID
  int lcnt;  // 縁石データのライン数

  // 個別地物毎のデータ変換
  for(MpDmpCurb_t::const_iterator it = g_MpDmpCurb.begin();
      it != g_MpDmpCurb.end(); ++it)
  {
    // 一連の座標データから、ADASラインデータに変換
    if(!tranDmpLineToAdasLines(it->second.se_geometry, slid, flid))
    {
      std::cerr << "ERROR:[tranDmpCurbToAdasCurb-01] tranDmpLineToAdasLines(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    lcnt = flid - slid + 1;  // 生成する縁石データの数

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToAdasLines(slid, flid);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpCurbToAdasCurb-02] searchAdasLaneNearestToAdasLine(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADAS縁石(R003)を格納
    for(int i=0; i<lcnt; i++)
    {
      insertAdasCurb(++c_CurbID,  // 縁石ID
                     slid + i,  // ラインクラスID (縁石は道路側の下ラインで取る)
                     0.1,    // 縁石ブロックの高さ
                     0.1,    // 縁石ブロックの厚み (厚みの向きは向きフィールドに従う)
                     0,      // 向き(進行方向 右:0、左:1)
                     lkid);  // 最寄りのレーンID

      // デバッグログ出力
      if(bDebugFlg)
      ofsdbg << "[FL8311]Curb=" << it->second.feature_id << ", i=" << i
            << " -> (R003)CurbID=" << c_CurbID << std::endl;
    }
  }

  return true;
}
#endif

#if 0
/************************************************************
*  [FA8312]側溝のデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADAS縁石(R003)へ変換
************************************************************/
static bool tranDmpGutterToAdasGutter(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpGutter_t::const_iterator it = g_MpDmpGutter.begin();
      it != g_MpDmpGutter.end(); ++it)
  {
    // 一連の座標データから、ADASエリアデータに変換
    int aid = 0;  // ADASエリアIDワーク
    if(!tranDmpAreaToAdasArea(it->second.sf_mbr, aid))
    {
      std::cerr << "ERROR:[tranDmpGutterToAdasGutter-01] tranDmpAreaToAdasArea(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    // 最寄りのADASレーンIDを検索
    int lkid = searchAdasLaneNearestToAdasArea(aid);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpGutterToAdasGutter-02] searchAdasLaneNearestToAdasArea(), "
                << "feature_id=" << it->second.itemID << std::endl;

    // ADAS側溝(R003)を格納
    insertAdasGutter(++c_GutterID,  // 側溝ID
                     aid,    // エリアクラスID
                     0,      // 種別
                     lkid);  // 最寄りのレーンID

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FA8312]Gutter=" << it->second.feature_id
             << " -> (R002)GutterID=" << c_GutterID;
  }

  return true;
}
#endif

#if 0
/************************************************************
*  [FA8313]ガードレールのデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADASガードレール(S001)へ変換
************************************************************/
static bool tranDmpGuardRailToAdasGuardrail(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpGuardRail_t::const_iterator it = g_MpDmpGuardRail.begin();
      it != g_MpDmpGuardRail.end(); ++it)
  {
    // 一連の座標データから、ADASエリアデータに変換
    int aid = 0;  // エリアID
    if(!tranDmpAreaToAdasArea(it->second.sf_mbr, aid))
    {
      std::cerr << "ERROR:[tranDmpGuardRailToAdasGutter-01] tranDmpAreaToAdasArea(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    // 最寄りのADASレーンIDを検索
    //int lkid = searchAdasLaneNearestToAdasArea(aid);
    if(lkid == 0)
      std::cerr << "WARNING:[tranDmpGuardRailToAdasGutter-02] searchAdasLaneNearestToAdasLines(), "
                << "feature_id=" << it->second.feature_id << std::endl;

    // ADASガードレール(S001)を格納
    insertAdasGuardrail(++c_GuardrailID, // 路面マークID
                       aid,  // エリアクラスID
                       0,    // 羽種別
                       0);   // 最寄りのレーンID

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FA8313]GuardRail=" << it->second.feature_id
             << " -> (S001)GuardrailID=" << c_GuardrailID;
  }

  return true;
}
#endif

#if 0
/************************************************************
*  [FA8314]ゼブラゾーンのデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
*    note: ADASゼブラゾーン(P003)へ変換
************************************************************/
static bool tranDmpZebraZoneToAdasZebrazone(void)
{
  // 個別地物毎のデータ変換
  for(MpDmpZebraZone_t::const_iterator it = g_MpDmpZebraZone.begin();
      it != g_MpDmpZebraZone.end(); ++it)
  {
    // 一連の座標データから、ADASエリアデータに変換
    int aid = 0;  // エリアID
    if(!tranDmpAreaToAdasArea(it->second.sf_mbr, aid))
    {
      std::cerr << "ERROR:[tranDmpZebraZoneToAdasZebrazone-01] tranDmpAreaToAdasArea(), "
                << "feature_id=" << it->second.feature_id << std::endl;
      return false;
    }

    // 最寄りのADASレーンIDを検索
    //int lkid = searchAdasLaneNearestToAdasArea(aid);
    //if(lkid == 0)
    //  std::cerr << "WARNING:[tranDmpZebraZoneToAdasZebrazone-02] searchAdasLaneNearestToAdasLines(), "
    //            << "itemID=" << it->second.itemID << std::endl;

    // ADASゼブラゾーン(P003)を格納
    insertAdasZebrazone(++c_CrosswalkID, // ゼブラゾーンID
                        aid,  // エリアクラスID
                        0);   // 最寄りのレーンID

    // デバッグログ出力
    if(bDebugFlg)
      ofsdbg << "[FA8314]ZebraZone=" << it->second.feature_id
             << " -> (P003)ZebrazoneID=" << c_ZebrazoneID;
  }

  return true;
}
#endif

/************************************************************
*  ADASレーンデータの繋ぎ込み処理
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool connectAdasLaneData(void)
{
  int inLaneFID;     // 進入側Dmpレーン地物IDバッファ
  int outLaneFID;    // 退出側Dmpレーン地物IDバッファ
  int inLnID;        // 進入側ADASレーンIDバッファ
  int outLnID;       // 退出側ADASレーンIDバッファ
  int lnbuff[4];     // レーンのソート用 レーンIDバッファ
  double anbuff[4];  // レーンのソート用 角度値バッファ
  int lncnt;

  // 経路接続の関連データによる繋ぎ込み処理
  for(VcDmpRebRelationship_t::const_iterator it1 = g_VcDmpRebRelationshipConnectivity.begin();
      it1 != g_VcDmpRebRelationshipConnectivity.end(); ++it1)
  {
    // 進入側(1:From Lane)を基準にして、退出側(4:To Lane)を接続していく
    for(VcDmpRebRelationshipFeat_t::const_iterator it2 = it1->mVcDmpRebRelationshipFeat.begin();
        it2 != it1->mVcDmpRebRelationshipFeat.end(); ++it2)
    {
      // 進入側(it2)レーンを得る
      if(it2->role_number == 1)  // 1:From Lane
      {
        inLaneFID = it2->feature_id;
        //std::cerr << "DEBUG:[connectAdasLaneData] in rel_id=" << it1->relationship_id << ", inLane=" << inLaneFID << std::endl;
        for(VcDmpRebRelationshipFeat_t::const_iterator it3 = it1->mVcDmpRebRelationshipFeat.begin();
            it3 != it1->mVcDmpRebRelationshipFeat.end(); ++it3)
        {
          // 退出側(it3)レーンを得る
          if(it3->role_number == 4)  // 4:To Lane
          {
            outLaneFID = it3->feature_id;

            // 進入側のADASレーンIDを得る
            MpLaneBgnEndBuff_t::iterator iti = mMpLaneBgnEndBuff.find(inLaneFID);
            if(iti == mMpLaneBgnEndBuff.end())
            {
              std::cerr << "ERROR:[connectAdasLaneData-01] illegal inLaneFID=" << inLaneFID << std::endl;
              return false;
            }
            inLnID = iti->second.endLnID;

            // 退出側のADASレーンIDを得る
            MpLaneBgnEndBuff_t::iterator ito = mMpLaneBgnEndBuff.find(outLaneFID);
            if(ito == mMpLaneBgnEndBuff.end())
            {
              std::cerr << "ERROR:[connectAdasLaneData-02] illegal outLaneFID=" << outLaneFID << std::endl;
              return false;
            }
            outLnID = ito->second.bgnLnID;

            // 退出側ADASレーンに進入側を接続
            MpAdLane_t::iterator itbl = g_MpAdLane.find(outLnID);
            if(itbl == g_MpAdLane.end())
            {
              std::cerr << "ERROR:[connectAdasLaneData-03] illegal LnID=" << outLnID << std::endl;
              return false;
            }
            if(itbl->second.BLID == 0)
              itbl->second.BLID = inLnID;
            else if(itbl->second.BLID2 == 0)
              itbl->second.BLID2 = inLnID;
            else if(itbl->second.BLID3 == 0)
              itbl->second.BLID3 = inLnID;
            else if(itbl->second.BLID4 == 0)
              itbl->second.BLID4 = inLnID;
            else
            {
              std::cerr << "ERROR:[connectAdasLaneData-04] BLID full, LnID=" << outLnID << std::endl;
              return false;
            }

            // 進入側ADASレーンに退出側を接続
            MpAdLane_t::iterator itfl = g_MpAdLane.find(inLnID);
            if(itfl == g_MpAdLane.end())
            {
              std::cerr << "ERROR:[connectAdasLaneData-05] illegal LnID=" << inLnID << std::endl;
              return false;
            }
            if(itfl->second.FLID == 0)
              itfl->second.FLID = outLnID;
            else if(itfl->second.FLID2 == 0)
              itfl->second.FLID2 = outLnID;
            else if(itfl->second.FLID3 == 0)
              itfl->second.FLID3 = outLnID;
            else if(itfl->second.FLID4 == 0)
              itfl->second.FLID4 = outLnID;
            else
            {
              std::cerr << "ERROR:[connectAdasLaneData-06] FLID full, LnID=" << inLnID << std::endl;
              return false;
            }

            // デバッグログ出力
            if(bDebugFlg)
              ofsdbg << "Connect ADAS Lane: " << inLnID << " -> " << outLnID << std::endl;
          }
        }
      }
    }
  }

  // 車線リンクの始点終点毎に、始点ADASレーンの合流関係と、終点ADASレーンの分岐関係を整理する
  for(MpLaneBgnEndBuff_t::const_iterator it = mMpLaneBgnEndBuff.begin();
      it != mMpLaneBgnEndBuff.end(); ++it)
  {
    // 始点ADASレーンの検索
    MpAdLane_t::iterator itbl = g_MpAdLane.find(it->second.bgnLnID);
    if(itbl == g_MpAdLane.end())
    {
      std::cerr << "ERROR:[connectAdasLaneData-07] illegal LnID=" << it->second.bgnLnID << std::endl;
      return false;
    }

    // ソート用バッファに合流するレーンを入れる
    lnbuff[0] = lnbuff[1] = lnbuff[2] = lnbuff[3] = 0;
    anbuff[0] = anbuff[1] = anbuff[2] = anbuff[3] = 0;
    lncnt = 0;
    if((itbl->second.BLID != 0) && (itbl->second.BLID2 != 0))
    {
      lnbuff[0] = itbl->second.BLID;
      anbuff[0] = calAngleCrossAdasLanes(it->second.bgnLnID, itbl->second.BLID);
      lnbuff[1] = itbl->second.BLID2;
      anbuff[1] = calAngleCrossAdasLanes(it->second.bgnLnID, itbl->second.BLID2);
      lncnt = 2;
      if(itbl->second.BLID3 > 0)
      {
        lnbuff[2] = itbl->second.BLID3;
        anbuff[2] = calAngleCrossAdasLanes(it->second.bgnLnID, itbl->second.BLID3);
        lncnt = 3;
        if(itbl->second.BLID4 > 0)
        {
          lnbuff[3] = itbl->second.BLID4;
          anbuff[3] = calAngleCrossAdasLanes(it->second.bgnLnID, itbl->second.BLID4);
          lncnt = 4;
        }
      }

      // 合流するレーンをソート
      bool cflg = true;
      while(cflg)
      {
        cflg = false;
        for(int i=0; i<(lncnt - 1); i++)
        {
          if(fabs(anbuff[i]) > fabs(anbuff[i + 1]))
          {
            std::swap(lnbuff[i], lnbuff[i + 1]);
            std::swap(anbuff[i], anbuff[i + 1]);
            cflg = true;
          }
        }
      }

      itbl->second.BLID = lnbuff[0];
      itbl->second.BLID2 = lnbuff[1];
      itbl->second.BLID3 = lnbuff[2];
      itbl->second.BLID4 = lnbuff[3];
      if(anbuff[0] < anbuff[1])
        itbl->second.JCT = 4;  // 左(3)右(4)から合流
      else
        itbl->second.JCT = 3;  // 左(3)右(4)から合流
    }

    // 終点ADASレーンの検索
    MpAdLane_t::iterator itel = g_MpAdLane.find(it->second.endLnID);
    if(itel == g_MpAdLane.end())
    {
      std::cerr << "ERROR:[connectAdasLaneData-08] illegal LnID=" << it->second.endLnID << std::endl;
      return false;
    }

    // ソート用バッファに分岐するレーンを入れる
    lnbuff[0] = lnbuff[1] = lnbuff[2] = lnbuff[3] = 0;
    anbuff[0] = anbuff[1] = anbuff[2] = anbuff[3] = 0;
    lncnt = 0;
    if((itel->second.FLID != 0) && (itel->second.FLID2 != 0))
    {
      lnbuff[0] = itel->second.FLID;
      anbuff[0] = calAngleCrossAdasLanes(it->second.endLnID , itel->second.FLID);
      lnbuff[1] = itel->second.FLID2;
      anbuff[1] = calAngleCrossAdasLanes(it->second.endLnID , itel->second.FLID2);
      lncnt = 2;
      if(itel->second.FLID3 > 0)
      {
        lnbuff[2] = itel->second.FLID3;
        anbuff[2] = calAngleCrossAdasLanes(it->second.endLnID , itel->second.FLID3);
        lncnt = 3;
        if(itel->second.FLID4 > 0)
        {
          lnbuff[3] = itel->second.FLID4;
          anbuff[3] = calAngleCrossAdasLanes(it->second.endLnID , itel->second.FLID4);
          lncnt = 4;
        }
      }

      // 分岐するレーンをソート
      bool cflg = true;
      while(cflg)
      {
        cflg = false;
        for(int i=0; i<(lncnt - 1); i++)
        {
          if(fabs(anbuff[i]) > fabs(anbuff[i + 1]))
          {
            std::swap(lnbuff[i], lnbuff[i + 1]);
            std::swap(anbuff[i], anbuff[i + 1]);
            cflg = true;
          }
        }
      }

      itel->second.FLID = lnbuff[0];
      itel->second.FLID2 = lnbuff[1];
      itel->second.FLID3 = lnbuff[2];
      itel->second.FLID4 = lnbuff[3];
      if(anbuff[0] < anbuff[1])
        itel->second.JCT = 1;  // 左(1)右(2)へ分岐
      else
        itel->second.JCT = 2;  // 左(1)右(2)へ分岐
    }
  }

  return true;
}

/************************************************************
*  ADASレーンデータが、ADAS交差点(R004)に含まれるかを調べて、
*  ADASレーンデータにADAS交差点IDを設定する
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool setAdasIntersectionToAdasLane(void)
{
  double x1, y1, z1, x2, y2, z2;

  // ADASレーンの検索
  for(MpAdLane_t::iterator it = g_MpAdLane.begin(); it != g_MpAdLane.end(); ++it)
  {
    if(it->second.CrossID == 9999999)
    {
      // ADASレーンの始点ノードと終点ノードの座標を求める
      if(!getAdasLanePoints(it->first, x1, y1, z1, x2, y2, z2))
      {
        std::cerr << "ERROR:[setAdasIntersectionToAdasLane-01] illegal LnID=" << it->first << std::endl;
        continue;
      }

      // 交差点領域拡張バッファを検索
      for(VcIntersectionAreaExBuff_t::const_iterator itc = mVcIntersectionAreaExBuff.begin();
          itc != mVcIntersectionAreaExBuff.end(); ++itc)
      {
        // 始点ノードの座標が交差点領域に含まれるかチェック
        if((x1 >= itc->xmin) && (x1 <= itc->xmax) &&
           (y1 >= itc->ymin) && (y1 <= itc->ymax))
        {
         if(checkInclusion(itc->pgn, x1, y1))
          {
            it->second.CrossID = itc->IsID;  // 対応するADAS交差点IDを設定
            break;
          }
        }

        // 終点ノードの座標が交差点領域に含まれるかチェック
        if((x2 >= itc->xmin) && (x2 <= itc->xmax) &&
           (y2 >= itc->ymin) && (y2 <= itc->ymax))
        {
          if(checkInclusion(itc->pgn, x2, y2))
          {
            it->second.CrossID = itc->IsID;  // 対応するADAS交差点IDを設定
            break;
          }
        }
      }

      if(it->second.CrossID == 9999999)
      {
        it->second.CrossID = 0;
        //std::cerr << "WARNING:[setAdasIntersectionToAdasLane-02] unmatch lnid=" << it->first << std::endl;
      }
    }
  }

  return true;
}

/************************************************************
*  ADAS標識と停止線とレーンの関連付け
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool linkAdasRoadsignStoplineLane(void)
{
  int fid_Lane;
  int fid_ExtendedTrafficSign;
  int fid_StopLine;
  int ad_LnID;
  int ad_RoadsignID;
  int ad_StoplineID;

  // 標識規制の関連データによる関連付け処理
  for(VcDmpRebRelationship_t::const_iterator it = g_VcDmpRebRelationshipTrafficSignRegulationForLane.begin();
      it != g_VcDmpRebRelationshipTrafficSignRegulationForLane.end(); ++it)
  {
    fid_Lane = 0;
    fid_ExtendedTrafficSign = 0;
    fid_StopLine = 0;
    ad_LnID = 0;
    ad_RoadsignID = 0;
    ad_StoplineID = 0;

    // 関連する地物を抽出
    for(VcDmpRebRelationshipFeat_t::const_iterator itf = it->mVcDmpRebRelationshipFeat.begin();
        itf != it->mVcDmpRebRelationshipFeat.end(); ++itf)
    {
      if(itf->role_number == 1)  // 1:Lane
        fid_Lane = itf->feature_id;
      else if(itf->role_number == 2)  // 2:ExtendedTrafficSign
        fid_ExtendedTrafficSign = itf->feature_id;
      else if(itf->role_number == 3)  // 3:StopLine
        fid_StopLine = itf->feature_id;
    }

    if(fid_ExtendedTrafficSign)
    {
      // ADAS標識IDの検索
      MpRoadsignExBuff_t::iterator itre = mMpRoadsignExBuff.find(fid_ExtendedTrafficSign);
      if(itre == mMpRoadsignExBuff.end())
      {
        std::cerr << "ERROR:[linkAdasRoadsignStoplineLane-01] illegal fid_ExtendedTrafficSign="
                  << fid_ExtendedTrafficSign << std::endl;
        return false;
      }
      ad_RoadsignID = itre->second.ad_RoadsignID;

      // ADASレーンIDの検索
      if(fid_Lane)
        ad_LnID = searchAdasLaneNearestToPointWithFid(itre->second.xRct, itre->second.yRct, itre->second.zHgt, fid_Lane);

      // ADAS停止線IDの検索
      MpStoplineExBuff_t::iterator itse = mMpStoplineExBuff.find(fid_StopLine);
      if(itse == mMpStoplineExBuff.end())
      {
        std::cerr << "ERROR:[linkAdasRoadsignStoplineLane-02] illegal fid_StopLine="
                  << fid_StopLine << std::endl;
        return false;
      }
      ad_StoplineID = itse->second.ad_StoplineID;

      // ADAS停止線に、ADAS標識IDとADASレーンIDを関連付け
      MpAdStopline_t::iterator itsl = g_MpAdStopline.find(ad_StoplineID);
      if(itsl == g_MpAdStopline.end())
      {
        std::cerr << "ERROR:[linkAdasRoadsignStoplineLane-03] illegal ad_StoplineID="
                  << ad_StoplineID << std::endl;
        return false;
      }
      itsl->second.SignID = ad_RoadsignID;
      itsl->second.LinkID = ad_LnID;

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "Link ADAS StoplineID: " << ad_StoplineID
               << " <- RoadsignID:" << ad_RoadsignID
               << " <- LnID:" << ad_LnID << std::endl;
    }
  }

  return true;
}

/************************************************************
*  ADAS信号と停止線とレーンの関連付け
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
static bool linkAdasSignalStoplineLane(void)
{
  int fid_Lane;
  int fid_ExtendedTrafficLight;
  int fid_StopLine;
  int ad_LnID;
  int ad_SignaldataID;
  int ad_StoplineID;

  // 信号規制の関連データによる関連付け処理
  for(VcDmpRebRelationship_t::const_iterator it = g_VcDmpRebRelationshipTrafficLightRegulationForLane.begin();
      it != g_VcDmpRebRelationshipTrafficLightRegulationForLane.end(); ++it)
  {
    fid_Lane = 0;
    fid_ExtendedTrafficLight = 0;
    fid_StopLine = 0;
    ad_LnID = 0;
    ad_SignaldataID = 0;
    ad_StoplineID = 0;

    // 関連する地物を抽出
    for(VcDmpRebRelationshipFeat_t::const_iterator itf = it->mVcDmpRebRelationshipFeat.begin();
        itf != it->mVcDmpRebRelationshipFeat.end(); ++itf)
    {
      if(itf->role_number == 1)  // 1:Lane
        fid_Lane = itf->feature_id;
      else if(itf->role_number == 2)  // 2:ExtendedTrafficLight
        fid_ExtendedTrafficLight = itf->feature_id;
      else if(itf->role_number == 3)  // 3:StopLine
        fid_StopLine = itf->feature_id;
    }

    if(fid_ExtendedTrafficLight)
    {
      // ADAS信号IDの検索
      MpSignaldataExBuff_t::iterator itsde = mMpSignaldataExBuff.find(fid_ExtendedTrafficLight);
      if(itsde == mMpSignaldataExBuff.end())
      {
        std::cerr << "ERROR:[linkAdasSignalStoplineLane-01] illegal fid_ExtendedTrafficLight="
                  << fid_ExtendedTrafficLight << std::endl;
        return false;
      }
      ad_SignaldataID = itsde->second.ad_SignalLamps[0].ad_SignaldataID;

      // ADASレーンIDの検索
      if(fid_Lane)
        ad_LnID = searchAdasLaneNearestToPointWithFid(itsde->second.ad_SignalLamps[0].xRct,
                    itsde->second.ad_SignalLamps[0].yRct, itsde->second.ad_SignalLamps[0].zHgt, fid_Lane);

      // ADAS停止線IDの検索
      MpStoplineExBuff_t::iterator itsle = mMpStoplineExBuff.find(fid_StopLine);
      if(itsle == mMpStoplineExBuff.end())
      {
        std::cerr << "ERROR:[linkAdasSignalStoplineLane-02] illegal fid_StopLine="
                  << fid_StopLine << std::endl;
        return false;
      }
      ad_StoplineID = itsle->second.ad_StoplineID;

      // ADAS停止線に、ADAS信号IDとADASレーンIDを関連付け
      MpAdStopline_t::iterator itsl = g_MpAdStopline.find(ad_StoplineID);
      if(itsl == g_MpAdStopline.end())
      {
        std::cerr << "ERROR:[linkAdasSignalStoplineLane-03] illegal ad_StoplineID="
                  << ad_StoplineID << std::endl;
        return false;
      }
      itsl->second.TLID = ad_SignaldataID;
      itsl->second.LinkID = ad_LnID;

      // ADAS信号に、ADASレーンIDを関連付け
      for(VcSignalLampExBuff_t::const_iterator itsdle = itsde->second.ad_SignalLamps.begin(); 
          itsdle != itsde->second.ad_SignalLamps.end(); ++itsdle)
      {
        MpAdSignaldata_t::iterator itsd = g_MpAdSignaldata.find(itsdle->ad_SignaldataID);
        if(itsd == g_MpAdSignaldata.end())
        {
          std::cerr << "ERROR:[linkAdasSignalStoplineLane-04] illegal ad_SignaldataID="
                    << itsdle->ad_SignaldataID << std::endl;
          return false;
        }
        itsd->second.LinkID = ad_LnID;
      }

      // デバッグログ出力
      if(bDebugFlg)
        ofsdbg << "Link ADAS StoplineID: " << ad_StoplineID
               << " <- SignaldataID:" << ad_SignaldataID
               << " <- LnID:" << ad_LnID << std::endl;
    }
  }

  return true;
}

/************************************************************
*  静的ダイナミックマップからADASマップへのデータ変換
*
*    input: nothing
*    output: nothing
*    return: リターンコード (true=成功、false=失敗)
************************************************************/
bool tranDmpToAdas(void)
{
  // ADASマップ作業領域の初期化
  c_PID = 0;
  c_VID = 0;
  c_LID = 0;
  c_AID = 0;
  c_PLID = 0;
  c_BID = 0;

  c_DID = 0;
  c_NID = 0;
  c_LnID = 0;
  c_WAID = 0;

  c_RoadedgeID = 0;
  c_GutterID= 0;
  c_CurbID = 0;
  c_IntersectionID = 0;
  c_SidestripID = 0;

  c_WhitelineID = 0;
  c_StoplineID = 0;
  c_ZebrazoneID = 0;
  c_CrosswalkID = 0;
  c_RoadmarkID = 0;

  c_GuardrailID = 0;
  c_SidewalkID = 0;
  c_DriveonID = 0;
  c_PoledataID = 0;
  c_UtilitypoleID = 0;
  c_RoadsignID = 0;
  c_SignaldataID = 0;
  c_StreetlightID = 0;
  c_CurvemirrorID = 0;
  c_WallID = 0;
  c_FenceID = 0;
  c_RailroadID = 0;

  mVcLaneExBuff.clear();
  mMpLaneBgnEndBuff.clear();
  mVcIntersectionAreaExBuff.clear();

  g_MpAdIndex.clear();
  g_MpAdPoint.clear();
  g_MpAdVector.clear();
  g_MpAdPole.clear();
  g_MpAdLine.clear();
  g_MpAdArea.clear();
  g_MpAdBox.clear();
  g_MpAdDtlane.clear();
  g_MpAdNode.clear();
  g_MpAdLane.clear();
  g_MpAdWayarea.clear();
  g_MpAdRoadedge.clear();
  g_MpAdGutter.clear();
  g_MpAdCurb.clear();
  g_MpAdIntersection.clear();
  g_MpAdSidestrip.clear();
  g_MpAdWhiteline.clear();
  g_MpAdStopline.clear();
  g_MpAdZebrazone.clear();
  g_MpAdCrosswalk.clear();
  g_MpAdRoadmark.clear();
  g_MpAdGuardrail.clear();
  g_MpAdSidewalk.clear();
  g_MpAdDriveon.clear();
  g_MpAdPoledata.clear();
  g_MpAdUtilitypole.clear();
  g_MpAdRoadsign.clear();
  g_MpAdSignaldata.clear();
  g_MpAdStreetlight.clear();
  g_MpAdCurvemirror.clear();
  g_MpAdWall.clear();
  g_MpAdFence.clear();
  g_MpAdRailroad.clear();

  // [FC8110]レーンの変換
  if(!tranDmpLaneToAdasLane())
  {
    std::cerr << "ERROR:[tranDmpToAdas-01] tranDmpLaneToAdasLane()" << std::endl;
    return false;
  }

#if 0
  // [FC8140]歩道の変換
  if(!tranDmpExtendedPedestrianToAdasSidewalk())
  {
    std::cerr << "ERROR:[tranDmpToAdas-02] tranDmpExtendedPedestrianToAdasSidewalk()" << std::endl;
    return false;
  }
#endif

  // [FC8145]横断歩道の変換
  if(!tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk())
  {
    std::cerr << "ERROR:[tranDmpToAdas-03] tranDmpExtendedPedestrianCrossingToAdasAdasCrosswalk()" << std::endl;
    return false;
  }

  // [FA8170]交差点領域の変換
  if(!tranDmpIntersectionAreaShapeToAdasIntersection())
  {
    std::cerr << "ERROR:[tranDmpToAdas-04] tranDmpIntersectionAreaShapeToAdasIntersection()" << std::endl;
    return false;
  }

  // [FP8210]ポールのデータ変換
  if(!tranDmpPoleToAdasPole())
  {
    std::cerr << "ERROR:[tranDmpToAdas-05] tranDmpPoleToAdasPole()" << std::endl;
    return false;
  }

  // [FC8220]標識のデータ変換
  if(!tranDmpExtendedTrafficSignToAdasRoadsign())
  {
    std::cerr << "ERROR:[tranDmpToAdas-06] tranDmpExtendedTrafficSignToAdasRoadsign()" << std::endl;
    return false;
  }

  // [FC8230]信号の変換
  if(!tranDmpExtendedTrafficLightToAdasSignaldata())
  {
    std::cerr << "ERROR:[tranDmpToAdas-07] tranDmpExtendedTrafficLightToAdasSignaldata()" << std::endl;
    return false;
  }

#if 0
  // [FC8240]街灯のデータ変換
  if(!tranDmpExtendedLightingToAdasStreetlight())
  {
    std::cerr << "ERROR:[tranDmpToAdas-08] tranDmpExtendedLightingToAdasStreetlight()" << std::endl;
    return false;
  }
#endif

#if 0
  // [FC8250]路面マークのデータ変換
  if(!tranDmpExtendedRoadMarkingsToAdasRoadmark())
  {
    std::cerr << "ERROR:[tranDmpToAdas-09] tranDmpExtendedRoadMarkingsToAdasRoadmark()" << std::endl;
    return false;
  }
#endif

  // [FL8260]停止線のデータ変換
  if(!tranDmpStopLineToAdasStopline())
  {
    std::cerr << "ERROR:[tranDmpToAdas-10] tranDmpStopLineToAdasStopline()" << std::endl;
    return false;
  }

#if 0
  // [FL8310]道路縁のデータ変換
  if(!tranDmpRoadEdgeToAdasRoadedge())
  {
    std::cerr << "ERROR:[tranDmpToAdas-11] tranDmpRoadEdgeToAdasRoadedge()" << std::endl;
    return false;
  }
#endif

#if 0
  // [FL8311]縁石のデータ変換
  if(!tranDmpCurbToAdasCurb())
  {
    std::cerr << "ERROR:[tranDmpToAdas-12] tranDmpCurbToAdasCurb()" << std::endl;
    return false;
  }
#endif

#if 0
  // [FA8312]側溝のデータ変換
  if(!tranDmpGutterToAdasGutter())
  {
    std::cerr << "ERROR:[tranDmpToAdas-13] tranDmpGutterToAdasGutter()" << std::endl;
    return false;
  }
#endif

#if 0
  // [FA8313]ガードレールのデータ変換
  if(!tranDmpGuardRailToAdasGuardrail())
  {
    std::cerr << "ERROR:[tranDmpToAdas-14] tranDmpGuardRailToAdasGuardrail()" << std::endl;
    return false;
  }
#endif

#if 0
  // [FA8314]ゼブラゾーンのデータ変換
  if(!tranDmpZebraZoneToAdasZebrazone())
  {
    std::cerr << "ERROR:[tranDmpToAdas-14] tranDmpZebraZoneToAdasZebrazone()" << std::endl;
    return false;
  }
#endif

  // ADASレーンデータの繋ぎ込み
  if(!connectAdasLaneData())
  {
    std::cerr << "ERROR:[tranDmpToAdas-15] connectAdasLaneData()" << std::endl;
    return false;
  }

  // ADASレーンデータに交差点IDを設定
  if(!setAdasIntersectionToAdasLane())
  {
    std::cerr << "ERROR:[tranDmpToAdas-16] setAdasIntersectionToAdasLane()" << std::endl;
    return false;
  }

  // ADAS標識と停止線とレーンの関連付け
  if(!linkAdasRoadsignStoplineLane())
  {
    std::cerr << "ERROR:[tranDmpToAdas-17] linkAdasRoadsignStoplineLane()" << std::endl;
    //return false;
  }

  // ADAS信号と停止線とレーンの関連付け
  if(!linkAdasSignalStoplineLane())
  {
    std::cerr << "ERROR:[tranDmpToAdas-18] linkAdasSignalStoplineLane()" << std::endl;
    //return false;
  }

  if(bDebugFlg)
  {
    ofsdbg << std::endl;
    ofsdbg << "[result ADAS-MAP data count]" << std::endl;
    ofsdbg << "  PID=" << c_PID << std::endl;
    ofsdbg << "  VID=" << c_VID << std::endl;
    ofsdbg << "  LID=" << c_LID << std::endl;
    ofsdbg << "  AID=" << c_AID << std::endl;
    ofsdbg << "  PLID=" << c_PLID << std::endl;
    ofsdbg << "  BID=" << c_BID << std::endl;

    ofsdbg << "  DID=" << c_DID << std::endl;
    ofsdbg << "  NID=" << c_NID << std::endl;
    ofsdbg << "  LnID=" << c_LnID << std::endl;
    ofsdbg << "  WAID=" << c_WAID << std::endl;

    ofsdbg << "  RoadedgeID=" << c_RoadedgeID << std::endl;
    ofsdbg << "  GutterID=" << c_GutterID << std::endl;
    ofsdbg << "  CurbID=" << c_CurbID << std::endl;
    ofsdbg << "  IntersectionID=" << c_IntersectionID << std::endl;
    ofsdbg << "  SidestripID=" << c_SidestripID << std::endl;

    ofsdbg << "  WhitelineID=" << c_WhitelineID << std::endl;
    ofsdbg << "  StoplineID=" << c_StoplineID << std::endl;
    ofsdbg << "  ZebrazoneID=" << c_ZebrazoneID << std::endl;
    ofsdbg << "  CrosswalkID=" << c_CrosswalkID << std::endl;
    ofsdbg << "  RoadmarkID=" << c_RoadmarkID << std::endl;

    ofsdbg << "  GuardrailID=" << c_GuardrailID << std::endl;
    ofsdbg << "  SidewalkID=" << c_SidewalkID << std::endl;
    ofsdbg << "  DriveonID=" << c_DriveonID << std::endl;
    ofsdbg << "  PoledataID=" << c_PoledataID << std::endl;
    ofsdbg << "  UtilitypoleID=" << c_UtilitypoleID << std::endl;
    ofsdbg << "  RoadsignID=" << c_RoadsignID << std::endl;
    ofsdbg << "  SignaldataID=" << c_SignaldataID << std::endl;
    ofsdbg << "  StreetlightID=" << c_StreetlightID << std::endl;
    ofsdbg << "  CurvemirrorID=" << c_CurvemirrorID << std::endl;
    ofsdbg << "  WallID=" << c_WallID << std::endl;
    ofsdbg << "  FenceID=" << c_FenceID << std::endl;
    ofsdbg << "  RailroadID=" << c_RailroadID << std::endl << std::endl;
  }

  return true;
}
