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
#ifndef _DMP_FILE_H_
#define _DMP_FILE_H_

#include <string>
#include "dmp_data.h"

/************************************************************
*  グローバル定数定義
************************************************************/
#define USE_DMLIB  1  // DMLibを使用する

/************************************************************
*  グローバル変数定義
************************************************************/
//-----------------------------
// 地物データ読み込みバッファ
//-----------------------------
// [FP4120]ジャンクション(Junction)
extern MpDmpJunction_t g_MpDmpJunction;

// [FC8110]レーン(Lane)
extern MpDmpLane_t g_MpDmpLane;

// [FC8140]歩道(ExtendedPedestrian)
extern MpDmpExtendedPedestrian_t g_MpDmpExtendedPedestrian;

// [FC8145]横断歩道(ExtendedPedestrianCrossing)
extern MpDmpExtendedPedestrianCrossing_t g_MpDmpExtendedPedestrianCrossing;

// [FA8170]交差点領域(IntersectionAreaShape)
extern MpDmpIntersectionAreaShape_t g_MpDmpIntersectionAreaShape;

// [FC8230]信号(ExtendedTrafficLight)
extern MpDmpExtendedTrafficLight_t g_MpDmpExtendedTrafficLight;

// [FP8210]ポール(Pole)
extern MpDmpPole_t g_MpDmpPole;

// [FC8220]標識(ExtendedTrafficSign)
extern MpDmpExtendedTrafficSign_t g_MpDmpExtendedTrafficSign;

// [FC8240]街灯
extern MpDmpExtendedLighting_t g_MpDmpExtendedLighting;

// [FC8250]路面マーク
extern MpDmpExtendedRoadMarkings_t g_MpDmpExtendedRoadMarkings;

// [FL8260]停止線(Stopline)
extern MpDmpStopLine_t g_MpDmpStopLine;

// [FL8310]道路縁(RoadEdge)定義
extern MpDmpRoadEdge_t g_MpDmpRoadEdge;

// [FL8311]縁石(Curb)定義
extern MpDmpCurb_t g_MpDmpCurb;

// [FA8312]側溝(Gutter)定義
extern MpDmpGutter_t g_MpDmpGutter;

// [FA8313]ガードレール(GuardRail)定義
extern MpDmpGuardRail_t g_MpDmpGuardRail;

// [FA8314]ゼブラゾーン(ZebraZone)定義
extern MpDmpZebraZone_t g_MpDmpZebraZone;

//-----------------------------
// 関連データ再構築バッファ
//-----------------------------
// [R2110]経路接続(Connectivity)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipConnectivity;

// [R9110]交差(Crossing)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipCrossing;

// [R9120]隣接(Adjacency)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipAdjacency;

// [R9130]分岐(Branch)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipBranch;

// [R9210]信号規制(TrafficLightRegulationForLane)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipTrafficLightRegulationForLane;

// [R9220]標識規制(TrafficSignRegulationForLane)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipTrafficSignRegulationForLane;

// [R9230]地物関連(FeatureAlongWithLane)
extern VcDmpRebRelationship_t g_VcDmpRebRelationshipFeatureAlongWithLane;

/************************************************************
*  グローバル関数定義
************************************************************/

bool readDmpData(std::string host, int port, std::string user, std::string pass, std::string dbase, int sno);
void checkDmpData(void);
bool rebuildFeatureData(void);
bool rebuildRelationshipData(void);

#endif // _DMC_FILE_H_
