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
#ifndef _ADAS_FILE_H_
#define _ADAS_FILE_H_

#include <string>
#include "adas_data.h"

/******************************
*  グローバル変数定義
******************************/
// インデックス
extern MpAdIndex_t g_MpAdIndex;

// ポイントデータ (B01)
extern MpAdPoint_t g_MpAdPoint;

// ベクトルデータ (B02)
extern MpAdVector_t g_MpAdVector;

// ポールデータ (B03)
extern MpAdPole_t g_MpAdPole;

// ラインデータ (B04)
extern MpAdLine_t g_MpAdLine;

// エリアデータ (B05)
extern MpAdArea_t g_MpAdArea;

// ボックスデータ (B06)
extern MpAdBox_t g_MpAdBox;

// 中心線形データ (N01)
extern MpAdDtlane_t g_MpAdDtlane;

// ノードデータ (N02)
extern MpAdNode_t g_MpAdNode;

// レーンデータ個別 (N03)
extern MpAdLane_t g_MpAdLane;

// 走行エリアデータ (N04)
extern MpAdWayarea_t g_MpAdWayarea;

// 道路縁 (R001)
extern MpAdRoadedge_t g_MpAdRoadedge;

// 側溝 (R002)
extern MpAdGutter_t g_MpAdGutter;

// 縁石 (R003)
extern MpAdCurb_t g_MpAdCurb;

// 交差点 (R004)
extern MpAdIntersection_t g_MpAdIntersection;

// 路肩コンクリート (R005)
extern MpAdSidestrip_t g_MpAdSidestrip;

// 白線 (P001)
extern MpAdWhiteline_t g_MpAdWhiteline;

// 停止線 (P002)
extern MpAdStopline_t g_MpAdStopline;

// ゼブラゾーン (P003)
extern MpAdZebrazone_t g_MpAdZebrazone;

// 横断歩道 (P004)
extern MpAdCrosswalk_t g_MpAdCrosswalk;

// 路面マーク (P005)
extern MpAdRoadmark_t g_MpAdRoadmark;

// ガードレール (S001)
extern MpAdGuardrail_t g_MpAdGuardrail;

// 歩道 (S002)
extern MpAdSidewalk_t g_MpAdSidewalk;

// 車両乗入部 (S003)
extern MpAdDriveon_t g_MpAdDriveon;

// ポール (K001)
extern MpAdPoledata_t g_MpAdPoledata;

// 電柱 (K002)
extern MpAdUtilitypole_t g_MpAdUtilitypole;

// 標識 (K003)
extern MpAdRoadsign_t g_MpAdRoadsign;

// 信号 (K004)
extern MpAdSignaldata_t g_MpAdSignaldata;

// 街灯 (K005)
extern MpAdStreetlight_t g_MpAdStreetlight;

// カーブミラー (K006)
extern MpAdCurvemirror_t g_MpAdCurvemirror;

// 壁面(K007)
extern MpAdWall_t g_MpAdWall;

// フェンス (K008)
extern MpAdFence_t g_MpAdFence;

// 踏切ゾーン(K009)
extern MpAdRailroad_t g_MpAdRailroad;

/******************************
*  グローバル関数定義
******************************/

bool writeAdasMapFiles(std::string dname);

void insertAdasIndex(std::string KIND, std::string fname);
void insertAdasPoint(int PID, double B, double L, double H, double Bx, double Ly,
                     int Ref, int MCODE1, int MCODE2, int MCODE3);

void insertAdasVector(int VID, int PID, double Hang, double Vang);
void insertAdasPole(int PLID, int VID, double Length, double Dim);
void insertAdasLine(int LID, int BPID, int FPID, int BLID, int FLID);
void insertAdasArea(int AID, int SLID, int ELID);
void insertAdasBox(int BID, int PID1, int PID2, int PID3, int PID4, double Height);

void insertAdasDtlene(int DID, double Dist, int PID, double Dir, double Apara, double r,
                      double slope, double cant, double LW, double RW);
void insertAdasNode(int NID, int PID);
void insertAdasLane(int LnID, int DID, int BLID, int FLID, int BNID, int FNID, int JCT,
                    int BLID2, int BLID3, int BLID4, int FLID2, int FLID3, int FLID4,
                    int ClossID, double Span, int LCnt, int Lno, int LaneType, int LimitVel,
                    int RefVel, int RoadSecID, int LaneChgFG, int LinkWAID);
void insertAdasWayarea(int WAID, int AID);

void insertAdasRoadedge(int ID, int LID, int LinkID);
void insertAdasGutter(int ID, int AID, int Type, int LinkID);
void insertAdasCurb(int ID, int LID, double Height, double Width, int Dir, int LinkID);
void insertAdasIntersection(int ID, int AID, int LinkID);
void insertAdasSidestrip(int ID, int LID, int LinkID);

void insertAdasWhiteline(int ID, int LID, double Width, std::string Color, int type, int LinkID);
void insertAdasStopline(int ID, int LID, int TLID, int SignID, int LinkID);
void insertAdasZebrazone(int ID, int AID, int LinkID);
void insertAdasCrosswalk(int ID, int AID, int Type, int BdID, int LinkID);
void insertAdasRoadmark(int ID, int AID, std::string Type, int LinkID);

void insertAdasGuardrail(int ID, int AID, int Type, int LinkID);
void insertAdasSidewalk(int ID, int AID, int LinkID);
void insertAdasDriveon(int ID, int AID, int LinkID);

void insertAdasPoledata(int ID, int PLID, int LinkID);
void insertAdasUtilitypole(int ID, int PLID, int LinkID);
void insertAdasRoadsign(int ID, int VID, int PLID, std::string Type, int LinkID);
void insertAdasSignaldata(int ID, int VID, int PLID, int Type, int LinkID);
void insertAdasStreetlight(int ID, int LID, int PLID, int LinkID);
void insertAdasCurvemirror(int ID, int VID, int PLID, std::string Type, int LinkID);
void insertAdasWall(int ID, int AID, int LinkID);
void insertAdasFence(int ID, int AID, int LinkID);
void insertAdasRailroad(int ID, int AID, int LinkID);

#endif // _ADAS_FILE_H_
