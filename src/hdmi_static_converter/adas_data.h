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
#ifndef _ADAS_DATA_H_
#define _ADAS_DATA_H_

#include <map>
#include <string>
#include <vector>

/****************************************
*  インデックス定義
****************************************/

typedef enum
{
  ADAS_NONE = 0,     // 0:NONE
  B01_Point,         // 1:ポイントデータ (B01)
  B02_Vector,        // 2:ベクトルデータ (B02)
  B03_Pole,          // 3:ポールデータ (B03)
  B04_Line,          // 4:ラインデータ (B04)
  B05_Area,          // 5:エリアデータ (B05)
  B06_Box,           // 6:ボックスデータ (B06)
  N01_Dtlane,        // 7:中心線形データ (N01)
  N02_Node,          // 8:ノードデータ (N02)
  N03_Lane,          // 9:レーンデータ個別 (N03)
  N04_Wayarea,       // 10:走行エリアデータ (N04)
  R001_Roadedge,     // 11:道路縁 (R001)
  R002_Gutter,       // 12:側溝 (R002)
  R003_Curb,         // 13:縁石 (R003)
  R004_Intersection, // 14:交差点 (R004)
  R005_Sidestrip,    // 15:路肩コンクリート (R005)
  P001_Whiteline,    // 16:白線 (P001)
  P002_Stopline,     // 17:停止線 (P002)
  P003_Zebrazone,    // 18:ゼブラゾーン (P003)
  P004_Crosswalk,    // 19:横断歩道 (P004)
  P005_Roadmark,     // 20:路面マーク (P005)
  S001_Guardrail,    // 21:ガードレール (S001)
  S002_Sidewalk,     // 22:歩道 (S002)
  S003_Driveon,      // 23:車両乗入部 (S003)
  K001_Poledata,     // 24:ポール (K001)
  K002_Utilitypole,  // 25:電柱 (K002)
  K003_Roadsign,     // 26:標識 (K003)
  K004_Signaldata,   // 27:信号 (K004)
  K005_Streetlight,  // 28:街灯 (K005)
  K006_Curvemirror,  // 29:カーブミラー (K006)
  K007_Wall,         // 30:壁面(K007)
  K008_Fence,        // 31:フェンス (K008)
  K009_Railroad      // 32:踏切ゾーン(K009)
} AdasKind_e;

/****************************************
*  インデックスデータ定義
****************************************/

// インデックスデータ(idx.csv)定義
typedef std::map<std::string, std::string> MpAdIndex_t;  // (識別子, ファイル名)

/****************************************
*  基本図形クラス定義
****************************************/

// ポイントデータ(B01-point.csv)定義
typedef struct
{
  //int PID;    // ポイントのID
  double B;    // 緯度 [deg] ([D.MMSS]から変換済)
  double L;    // 経度 [deg] ([D.MMSS]から変換済)
  double H;    // 標高 [m]
  double Bx;   // 平面直角座標系 X座標 [m] (北向き、PC画面上では-y方向)
  double Ly;   // 平面直角座標系 Y座標 [m] (東向き、PC画面上ではx方向)
  int Ref;     // 平面直角座標系 座標系番号
  int MCODE1;  // 1次メッシュコード 基準地域メッシュの1次コード 4桁
  int MCODE2;  // 2次メッシュコード 基準地域メッシュの2次コード 2桁
  int MCODE3;  // 3次メッシュコード 基準地域メッシュの3次コード 2桁
} StAdPoint_t;
typedef std::map<int, StAdPoint_t> MpAdPoint_t;

// ベクトルデータ(B02-vector.csv)定義
typedef struct
{
  //int VID;    // ベクトルのID
  int PID;      // 起点ポイントID 起点の位置
  double Hang;  // 水平角 北0°時計回り [deg] ([D.MMSS]から変換済)
  double Vang;  // 鉛直角 天頂0°下回り [deg] ([D.MMSS]から変換済)
} StAdVector_t;
typedef std::map<int, StAdVector_t> MpAdVector_t;

// ポールデータ(B03-pole.csv)定義
typedef struct
{
  //int PLID;  // ポールのID
  int VID;     // 起点ベクトルID (ポールの起点と向き)
  double Length;  // 起点からの長さ [m]
  double Dim;     // 直径 [m]
} StAdPole_t;
typedef std::map<int, StAdPole_t> MpAdPole_t;

// ラインデータ(B04-line.csv)定義
typedef struct
{
  //int LID;  // ラインのID
  int BPID;  // 始点ポイントID (線分の1点目)
  int FPID;  // 終点ポイントID (線分の2点目)
  int BLID;  // 前のラインID (0:この線分が最初の線分)
  int FLID;  // 次のラインID (0:この線分が終端の線分)
} StAdLine_t;
typedef std::map<int, StAdLine_t> MpAdLine_t;

// エリアデータ(B05-area.csv)定義
typedef struct
{
  //int AID;  // エリアのID
  int SLID;  // 最初のラインID
  int ELID;  // 最後のラインID
} StAdArea_t;
typedef std::map<int, StAdArea_t> MpAdArea_t;

// ボックスデータ(B06-box.csv)定義
typedef struct
{
  //int BID;  // ボックスのID
  int PID1;  // 底面1点目ポイントID
  int PID2;  // 底面2点目ポイントID
  int PID3;  // 底面3点目ポイントID
  int PID4;  // 底面4点目ポイントID
  double Height;  // 高さ [m] (高さの向きは、底面左回りを表として表側に伸びる)
} StAdBox_t;
typedef std::map<int, StAdBox_t> MpAdBox_t;

/****************************************
*  道路データ定義
****************************************/

// 中心線形データ(N01-dtlane.csv)定義
typedef struct
{
  //int DID;  // レコードID
  double Dist;  // 追加距離 起点からの追加距離(起点0)
  int PID;       // 位置ポイントID 軌道ポイント位置
  double Dir;    // 方向角 [ラジアン単位]
  double Apara;  // クロソイドパラメータ (単曲線、直線は0)
  double r;      // 回転半径 (時計回りマイナス、直線区間、クロソイド起点は9.0E+10)
  double slope;  // 縦断勾配 [%表現]
  double cant;   // 横断勾配 [%表現]
  double LW;     // 左幅員
  double RW;     // 右幅員
                  // ※ 車線単位、道路単位どちらでも設定可
} StAdDtlane_t;
typedef std::map<int, StAdDtlane_t> MpAdDtlane_t;

// ノードデータ(N02-node.csv)定義
typedef struct
{
  //int NID;  // ノードID
  int PID;    // 位置ポイントID ポイントクラスのID
} StAdNode_t;
typedef std::map<int, StAdNode_t> MpAdNode_t;

// レーンデータ(N03-lane.csv)定義
typedef struct
{
  //int LnID;  // レーンID
  int DID;    // 中心線形ID
  int BLID;   // 手前のレコードID (始点データは0)
  int FLID;   // 次のレコードID (終点データは0)
  int BNID;   // 始点ノードID (この線分の始点ノードID)
  int FNID;   // 終点ノードID (この線分の終点ノードID)
  int JCT;    // 分岐合流パターン 通常区間(0)、左(1)右(2)へ分岐、左(3)右(4)から合流
  int BLID2;  // 合流ID2 (BLID以外にこのレーンに合流するレーンID)
  int BLID3;  // 合流ID3 (BLID以外にこのレーンに合流するレーンID)
  int BLID4;  // 合流ID4 (BLID以外にこのレーンに合流するレーンID)
  int FLID2;  // 分岐ID2 (FLID以外にこのレーンから分岐するレーンID)
  int FLID3;  // 分岐ID3 (FLID以外にこのレーンから分岐するレーンID)
  int FLID4;  // 分岐ID4 (FLID以外にこのレーンから分岐するレーンID)
  int CrossID;  // 交差点ID (このレコードが交差点に含まれる場合、そのID。通常は0)
  double Span;  // 区間距離 (この線分の長さ)
  int LCnt;     // 車線数 (交差点内は0)
  int Lno;      // レーン番号 (左から1、交差点内は0)
  int LaneType;  // レーン種別 (0:直進レーン、1:左折レーン、2:右折レーン。料金所レーン等もここで区別させる)
  int LimitVel;  // 法定速度
  int RefVel;    // 目標速度 (走行時の想定速度。現在は、法定速度以内の最大速度を設定)
  int RoadSecID;  // 道路区間ID (交差点内は0。隣り合うレーンは、通常同じ区間IDを持つ)
  int LaneChgFG;  // 車線変更可/不可 (0:可、1:不可、交差点内は0)
  int LinkWAID;  // 属する走行エリアID
} StAdLane_t;
typedef std::map<int, StAdLane_t> MpAdLane_t;

// 走行エリア(N04-wayarea.csv)定義 (Ver.1.20追加)
typedef struct
{
  //int WAID;  // 走行エリアID
  int AID;  // エリアID　
} StAdWayarea_t;
typedef std::map<int, StAdWayarea_t> MpAdWayarea_t;

/****************************************
*  地物(道路形状)定義
****************************************/

// 道路縁(R001-roadedge.csv)定義
typedef struct
{
  //int ID;    // ID
  int LID;     // ラインクラスID
  int LinkID;  // 最寄りのレーンID
} StAdRoadedge_t;
typedef std::map<int, StAdRoadedge_t> MpAdRoadedge_t;

// 側溝(R002-gutter.csv)定義
typedef struct
{
  //int ID;    // ID
  int AID;     // エリアクラスID (この側溝が占める領域)
  int Type;    // 種別 (0:蓋なし 1:フタあり 2:グレーチング)
  int LinkID;  // 最寄りのレーンID
} StAdGutter_t;
typedef std::map<int, StAdGutter_t> MpAdGutter_t;

// 縁石(R003-curb.csv)定義
typedef struct
{
  //int ID;  // ID
  int LID;   // ラインクラスID (縁石は道路側の下ラインで取る)
  double Height;  // 縁石ブロックの高さ
  double Width;   // 縁石ブロックの厚み (厚みの向きは向きフィールドに従う)
  int Dir;        // 向き(進行方向 右:0、左:1)
  int LinkID;     // 最寄りのレーンID
} StAdCurb_t;
typedef std::map<int, StAdCurb_t> MpAdCurb_t;

// 交差点(R004-intersection.csv)定義
typedef struct
{
  //int ID;    // ID
  int AID;     // エリアクラスID (交差点エリア)
  int LinkID;  // 最寄りのレーンID
} StAdIntersection_t;
typedef std::map<int, StAdIntersection_t> MpAdIntersection_t;

// 路肩コンクリート(R005-sidestrip.csv)定義
typedef struct
{
  //int ID;    // ID
  int LID;     // ラインクラスID
  int LinkID;  // 最寄りのレーンID
} StAdSidestrip_t;
typedef std::map<int, StAdSidestrip_t> MpAdSidestrip_t;

/****************************************
*  地物(路面)定義
****************************************/
// 白線(P001-whiteline.csv)定義
typedef struct
{
  //int ID;    // ID
  int LID;     // ラインクラスID (ラインは白線の中央を取る。)
  double Width;  // 白線太さ [m]
  std::string Color;  // 色 (W:白、Y:黄色)
  int Type;    // 種別 (0:実線、1:破線(実線)、2:破線(空白))
  int LinkID;  // 最寄りのレーンID
} StAdWhiteline_t;
typedef std::map<int, StAdWhiteline_t> MpAdWhiteline_t;

// 停止線(P002-stopline.csv)定義
typedef struct
{
  //int ID;    // ID
  int LID;     // ラインクラスID
  int TLID;    // 信号ランプID (関連する信号ランプのID(赤信号を想定)、0も可)
  int SignID;  // 標識ID (関連する標識のID(止まれ標識を想定)、0も可)
  int LinkID;  // 最寄りのレーンID
} StAdStopline_t;
typedef std::map<int, StAdStopline_t> MpAdStopline_t;

// ゼブラゾーン(P003-zebrazone.csv)定義
typedef struct
{
  //int ID;    // ID
  int AID;     // エリアクラスID (ゼブラゾーンの外形を取る)
  int LinkID;  // 最寄りのレーンID
} StAdZebrazone_t;
typedef std::map<int, StAdZebrazone_t> MpAdZebrazone_t;

// 横断歩道(P004-crosswalk.csv)定義
typedef struct
{
  //int ID;   // ID
  int AID;    // エリアクラスID (ゼブラゾーンの外形を取る)
  int Type;   // 種別 (0:外枠、1:縞模様、2:自転車通行帯)
  int BdID;   // 外枠ID 縞模様の場合、外枠のID。外枠の場合は0(自転車帯も0))
  int LinkID; // 最寄りのレーンID
} StAdCrosswalk_t;
typedef std::map<int, StAdCrosswalk_t> MpAdCrosswalk_t;

// 路面マーク(P005-road_surface_mark.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (矢印、文字等をフチ取り図形で取る)
  std::string Type;  // 種別 (矢印、高 等のこのマークが表す意味(memo)) ★★★ 実装はint
                     // (種別を数値コードで表す場合は 1:マーク、2:矢印、3:文字、4:記号)
  int LinkID;  // 最寄りのレーンID
} StAdRoadmark_t;
typedef std::map<int, StAdRoadmark_t> MpAdRoadmark_t;

/****************************************
*  地物(側帯)定義
****************************************/
// ガードレール(S001-guardrail.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (羽根の上端、下端で面にしたポリゴンエリア)
  int Type;  // 羽種別 (0:板羽根、1:ポール)
  int LinkID;  // 最寄りのレーンID
} StAdGuardrail_t;
typedef std::map<int, StAdGuardrail_t> MpAdGuardrail_t;

// 歩道(S002-sidewalk.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (歩道エリア)
  int LinkID;  // 最寄りのレーンID
} StAdSidewalk_t;
typedef std::map<int, StAdSidewalk_t> MpAdSidewalk_t;

// 車両乗入部(S003-driveon_portion.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (車道の乗り入れるエリア)
  int LinkID;  // 最寄りのレーンID
} StAdDriveon_t;
typedef std::map<int, StAdDriveon_t> MpAdDriveon_t;

/****************************************
*  地物(構造物)定義
****************************************/
// ポール(K001-poledata.csv)定義
typedef struct
{
  //int ID;  // ID
  int PLID;  // ポールクラスID
  int LinkID;  // 最寄りのレーンID 最寄りのレーンへの参照
} StAdPoledata_t;
typedef std::map<int, StAdPoledata_t> MpAdPoledata_t;

// 電柱(K002-utilitypole.csv)定義
typedef struct
{
  //int ID;  // ID
  int PLID;  // ポールクラスID
  int LinkID;  // 最寄りのレーンID
} StAdUtilitypole_t;
typedef std::map<int, StAdUtilitypole_t> MpAdUtilitypole_t;

// 標識(K003-roadsign.csv)定義
typedef struct
{
  //int ID;  // ID
  int VID;   // ベクトルクラスID (標識の中心から標識の向いている向きへベクトル)
  int PLID;  // ポールクラスID (この標識のポール、0も可)
  std::string Type;  // 種別 (予約) ★★★ 実装はint
  int LinkID;  // 最寄りのレーンID
} StAdRoadsign_t;
typedef std::map<int, StAdRoadsign_t> MpAdRoadsign_t;

// 信号(K004-signaldata.csv)定義
typedef struct
{
  //int ID;  // ID
  int VID;   // ベクトルクラスID (ランプの中心からランプの向いている方向へベクトル)
  int PLID;  // ポールクラスID (この信号ランプのポール、0も可)
  int Type;  // 種別 (赤(1),青(2),黄(3),歩行者赤(4),歩行者青(5),その他(9))
  int LinkID;  // 最寄りのレーンID
} StAdSignaldata_t;
typedef std::map<int, StAdSignaldata_t> MpAdSignaldata_t;

// 街灯(K005-streetlight.csv)定義
typedef struct
{
  //int ID;  // ID
  int LID;   // ラインクラスID (街灯ランプをラインで取る)
  int PLID;  // ポールクラスID (この街灯のポール、0も可)
  int LinkID;  // 最寄りのレーンID
} StAdStreetlight_t;
typedef std::map<int, StAdStreetlight_t> MpAdStreetlight_t;

// カーブミラー(K006-curvemirror.csv)定義
typedef struct
{
  //int ID;  // ID
  int VID;   // ベクトルクラスID (標識の中心から標識の向いている向きへベクトル)
  int PLID;  // ポールクラスID (この標識のポール)
  std::string Type;  // 種別 (予約) ★★★ 実装はint
  int LinkID;  // 最寄りのレーンID
} StAdCurvemirror_t;
typedef std::map<int, StAdCurvemirror_t> MpAdCurvemirror_t;

// 壁面(K007-wall.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (壁1枚)
  int LinkID;  // 最寄りのレーンID
} StAdWall_t;
typedef std::map<int, StAdWall_t> MpAdWall_t;

// フェンス(K008-fence.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (フェンス面)
  int LinkID;  // 最寄りのレーンID 最寄りのレーンへの参照
} StAdFence_t;
typedef std::map<int, StAdFence_t> MpAdFence_t;

// 踏切ゾーン(K009-railroad_crossing.csv)定義
typedef struct
{
  //int ID;  // ID
  int AID;   // エリアクラスID (踏切)
  int LinkID;  // 最寄りのレーンID
} StAdRailroad_t;
typedef std::map<int, StAdRailroad_t> MpAdRailroad_t;

#endif // _ADAS_DATA_H_
