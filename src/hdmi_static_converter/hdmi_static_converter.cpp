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
*  hdmi_static_converter.cpp
*  データベース形式ダイナミックマップを読み込み、
*  ベクタマップに変換して配信するメインプログラム
****************************************/

#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "dmp_file.h"
#include "adas_file.h"
#include "dmp_to_adas.h"

/************************************************************
*  グローバル変数定義
************************************************************/
bool bDebugFlg = false;  // デバッグログ出力フラグ
std::ofstream ofsdbg;    // デバッグログ出力ストリーム

/************************************************************
*  ローカル定数定義
************************************************************/
#define HOST_DEFAULT  "127.0.0.1"  // DBシステム／PostgreSQLサーバ ホスト名
#ifdef USE_DMLIB
 #define PORT_DEFAULT  9001        // DBシステム ポート番号
#else
 #define PORT_DEFAULT  5432        // PostgreSQLサーバ ポート番号
#endif
#define USER_DEFAULT  "dm2"        // PostgreSQLサーバ ユーザ名 
#define PASS_DEFAULT  "dm2"        // PostgreSQLサーバ パスワード 
#define DTBS_DEFAULT  "dm2_db"     // PostgreSQLデータベース名
#define ADAS_DEFAULT  "ADAS"       // ADAS-MAPを出力するディレクトリのパス
#define SYNO_DEFAULT  7            // 平面直角座標系指定 (n=1-19)

//#define ENABLE_TIME_MEASURE  1  // 実行時間測定用

/***********************************************************
*
*  データベース形式ダイナミックマップ 静的マップデータ変換プログラム メイン関数 ver 1.1
*    データベース形式ダイナミックマップの静的マップデータを読み込み、
*    ベクタマップ(ADAS-MAP)へと変換して、Autowareに配信する
*
*    input: args - コマンドライン引数
*#ifdef USE_DMLIB
*                    ・"-hn hostname" = DBシステム ホスト名指定 (default:localhost)
*                    ・"-pn portno"   = DBシステム ポート番号指定 (default:9001) 
*#else
*                    ・"-hn hostname" = PostgreSQLホスト名指定 (default:localhost)
*                    ・"-pn portno"   = PostgreSQLポート番号指定 (default:5432) 
*                    ・"-un username" = PostgreSQLユーザ名指定  (default:dm2)
*                    ・"-pw password" = PostgreSQLパスワード指定 (default:dm2)
*                    ・"-db database" = PostgreSQLデータベース名指定 (default:dm2_db)
*#endif
*                    ・"-sn" = 平面直角座標系指定 (n=1-19, default:7)
*                    ・"-pb" = 変換後Vectormap配信 (default:false)
*                    ・"-d" = デバッグログ出力 (debug.log, default:false)
*                    ・"-e" = エラーログ出力 (error.log, default:screen)
*                    ・"-h" = ヘルプ表示
*    output: ADASディレクトリ - 変換したADAS-MAPファイルの出力ディレクトリ
*    return: リターンコード
*
*    author: N.Iwasaki @AXE corp.
*    date written:  2018/Mar/22
*    last modified: 2018/Mar/30
*    Copyright 2018 AXE corp. All rights reserved.
***********************************************************/
int main(int argc, char *argv[])
{
  bool bErrorFlg = false;  // エラーログ出力フラグ
  std::ofstream ofserr;    // エラーログ出力ストリーム
  std::streambuf *cerrSave;

  std::string sHostName = HOST_DEFAULT;  // DBシステム／PostgreSQL ホスト名
  int         iPortNo   = PORT_DEFAULT;  // DBシステム／PostgreSQL ポート番号
  std::string sUserName = USER_DEFAULT;  // PostgreSQL ユーザ名
  std::string sPassword = PASS_DEFAULT;  // PostgreSQL パスワード
  std::string sDatabase = DTBS_DEFAULT;  // PostgreSQL データベース名
  int iSysNo = SYNO_DEFAULT;  // 平面直角座標系
  bool bPublishFlg = false;   // ベクタマップ配信フラグ
  const char *sKillCmd = "killall vector_map_loader > /dev/null 2>&1";  // ベクタマップ配信ROSノード停止コマンド

  std::cout << "Harmoware-DMI static-dynamicmap converter ver 1.1: START" << std::endl;

#ifdef ENABLE_TIME_MEASURE
  time_t timeStart = time(NULL);
#endif

  // コマンド引数の解析
  for(int i=1; i<argc; i++)
  {
    std::string option = argv[i];

    if((option == "-HN") || (option == "-hn"))
    {
      // DBシステム／PostgreSQLホスト名指定
      if(++i < argc)
        sHostName = argv[i];
      else
        std::cerr << "WARNING:[main-01] missing host name" << std::endl;
    }
    else if((option == "-PN") || (option == "-pn"))
    {
      // DBシステム／PostgreSQLポート番号指定
      if(++i < argc)
      {
        int ip = atoi(argv[i]);
        if((ip >= 1024) && (ip <= 65535))
          iPortNo = ip;
        else
          std::cerr << "WARNING:[main-02] illegal port number=" << std::string(argv[i]) << std::endl;
      }
      else
        std::cerr << "WARNING:[main-03] missing port number" << std::endl;
    }
    else if((option == "-UN") || (option == "-un"))
    {
      // PostgreSQLユーザ名指定
      if(++i < argc)
        sUserName = argv[i];
      else
        std::cerr << "WARNING:[main-04] missing user name" << std::endl;
    }
    else if((option == "-PW") || (option == "-pw"))
    {
      // PostgreSQLパスワード指定
      if(++i < argc)
        sPassword = argv[i];
      else
        std::cerr << "WARNING:[main-05] missing password" << std::endl;
    }
    else if((option == "-DB") || (option == "-db"))
    {
      // PostgreSQLデータベース名指定
      if(++i < argc)
        sDatabase = argv[i];
      else
        std::cerr << "WARNING:[main-05] missing database" << std::endl;
    }
    else if(((option.substr(0, 2) == "-S") || (option.substr(0, 2) == "-s")) &&
            (option.size() >= 3) && (option.size() <= 4))
    {
      // 平面直角座標系指定
      int sn = atoi(option.c_str() + 2);
      if((sn >= 1) && (sn <= 19))
      {
        iSysNo = sn;
        std::cout << "DEBUG:[main] Plane rectangular coordinate system=" << sn << std::endl;
      }
      else
      {
        iSysNo = 1000;
        std::cerr << "WARNING:[main-06] illegal coordinate system=" << sn << std::endl;
      }
    }
    else if((option == "-PB") || (option == "-pb"))
    {
      // 変換後Vectormap配信
      bPublishFlg = true;
      std::cout << "DEBUG:[main] Publish vectormap" << std::endl;
    }
    else if((option == "-D") || (option == "-d"))
    {
      // デバッグログ出力
      bDebugFlg = true;
      std::cout << "DEBUG:[main] Debug log output to <debug.log>" << std::endl;
    }
    else if((option == "-E") || (option == "-e"))
    {
      // エラーログ出力
      bErrorFlg = true;
      std::cout << "DEBUG:[main] Error log output to <error.log>" << std::endl;
    }
    else if((option == "-H") || (option == "-h"))
    {
      // ヘルプメッセージ出力
      std::cout
        << std::endl
        << "NAME" << std::endl
        << "    hdmi_static_converter - translate from static dynamic-map to vectormap." << std::endl
        << std::endl
        << "SYNOPSIS" << std::endl
#ifdef USE_DMLIB
        << "    hdmi_static_converter [-hn name] [-pn nnnn] [-sn] [pb] [-d] [-e] [-h]" << std::endl
#else
        << "    hdmi_static_converter [-hn name] [-pn nnnn] [-un name] [-pw pass] [-db name] [-sn] [pb] [-d] [-e] [-h]" << std::endl
#endif
        << std::endl
        << "DESCRIPTION"
        << "    translate from static dynamic-map to vectormap(ADAS-MAP)." << std::endl
        << std::endl
#ifdef USE_DMLIB
        << "      -hn name  set DBsystem hostname (default=localhost)" << std::endl
        << "      -pn nnnn  set DBsystem portnumber (default=9001)" << std::endl
#else
        << "      -hn name  set PostgreSQL hostname (default=localhost)" << std::endl
        << "      -pn nnnn  set PostgreSQL portnumber (default=5432)" << std::endl
        << "      -un name  set PostgreSQL username (default=dm2)" << std::endl
        << "      -pw pass  set PostgreSQL password (default=dm2)" << std::endl
        << "      -db name  set PostgreSQL database (default=dm2_db)" << std::endl
#endif
        << "      -sn       set Plane rectangular coordinate system (n=1-19, default=7)" << std::endl
        << "      -pb       publish vector map (default:false)" << std::endl
        << "      -d        output debug log to 'debug.log'. (default=none)" << std::endl
        << "      -e        output error log to 'error.log'. (default=console)" << std::endl
        << "      -h        display help(this) message." << std::endl
        << std::endl;
      return 0;
    }
    else
    {
      std::cerr << "ERROR:[main-07] illegal option=" << option << std::endl;
      return 1;
    }
  }

  if(bDebugFlg)
  {
    // デバッグログ出力ストリームのオープン
    ofsdbg.open("debug.log");
    time_t timer = time(NULL);
    ofsdbg << "DEBUG:[main] DEBUG LOG START at " << ctime(&timer) << std::endl;
  }

  if(bErrorFlg)
  {
    // エラーログ出力ストリームのオープン（エラー出力をコンソールからファイルに切り替え）
    ofserr.open("error.log");
    cerrSave = std::cerr.rdbuf();
    std::cerr.rdbuf(ofserr.rdbuf());
    time_t timer = time(NULL);
    std::cerr << "DEBUG:[main] ERROR LOG START at " << ctime(&timer) << std::endl;
  }

  if(bPublishFlg)
  {
    // もしROSノードが起動していれば、ノードを停止
    system(sKillCmd);
  }

  // 静的ダイナミックマップの読み込み
  if(!readDmpData(sHostName, iPortNo, sUserName, sPassword, sDatabase, iSysNo))
  {
    std::cerr << "ERROR:[main-08] readDmpData()" << std::endl;
    return -1;
  }

  // 地物データの再構築
  if(!rebuildFeatureData())
  {
    std::cerr << "ERROR:[main-09] rebuildFeatureData()" << std::endl;
    return -1;
  }
  // 関連データの再構築
  if(!rebuildRelationshipData())
  {
    std::cerr << "ERROR:[main-10] rebuildRelationshipData()" << std::endl;
    return -1;
  }
#ifdef ENABLE_TIME_MEASURE
  time_t timeRead = time(NULL);
#endif

  // 静的ダイナミックマップの読み込みデータの確認表示
  checkDmpData();

  // 静的ダイナミックマップのデータをADAS-MAPのデータに変換
  if(!tranDmpToAdas())
  {
    std::cerr << "ERROR:[main-11] tranDmpToAdas()" << std::endl;
    return -1;
  }
#ifdef ENABLE_TIME_MEASURE
  time_t timeTrans = time(NULL);
#endif

  // 変換したADAS-MAPの書き込み
  if(!writeAdasMapFiles("ADAS"))
  {
    std::cerr << "ERROR:[main-12] writeAdasMapFiles()" << std::endl;
    return -1;
  }
#ifdef ENABLE_TIME_MEASURE
  time_t timeWrite = time(NULL);
#endif

  if(bPublishFlg)
  {
    // ベクタマップ配信ROSノード起動コマンドの作成
    std::string sStartCmd;  // 起動コマンド
    std::string sCwd;  // カレントディレクトリのパス
    char sBuff[256];
    getcwd(sBuff, sizeof(sBuff));
    sCwd = std::string(sBuff);

    sStartCmd = "rosrun map_file vector_map_loader";
    if(!g_MpAdArea.empty())
      sStartCmd += " '" + sCwd + "/ADAS/area.csv'";
    if(!g_MpAdCrosswalk.empty())
      sStartCmd += " '" + sCwd + "/ADAS/crosswalk.csv'";
    if(!g_MpAdDtlane.empty())
      sStartCmd += " '" + sCwd + "/ADAS/dtlane.csv'";
    sStartCmd += " '" + sCwd + "/ADAS/idx.csv'";
    if(!g_MpAdIntersection.empty())
      sStartCmd += " '" + sCwd + "/ADAS/intersection.csv'";
    if(!g_MpAdLane.empty())
      sStartCmd += " '" + sCwd + "/ADAS/lane.csv'";
    if(!g_MpAdLine.empty())
      sStartCmd += " '" + sCwd + "/ADAS/line.csv'";
    if(!g_MpAdNode.empty())
      sStartCmd += " '" + sCwd + "/ADAS/node.csv'";
    if(!g_MpAdPoint.empty())
      sStartCmd += " '" + sCwd + "/ADAS/point.csv'";
    if(!g_MpAdPole.empty())
      sStartCmd += " '" + sCwd + "/ADAS/pole.csv'";
    if(!g_MpAdPoledata.empty())
      sStartCmd += " '" + sCwd + "/ADAS/poledata.csv'";
    if(!g_MpAdRoadmark.empty())
      sStartCmd += " '" + sCwd + "/ADAS/road_surface_mark.csv'";
    if(!g_MpAdRoadedge.empty())
      sStartCmd += " '" + sCwd + "/ADAS/roadedge.csv'";
    if(!g_MpAdRoadsign.empty())
      sStartCmd += " '" + sCwd + "/ADAS/roadsign.csv'";
    if(!g_MpAdSidewalk.empty())
      sStartCmd += " '" + sCwd + "/ADAS/sidewalk.csv'";
    if(!g_MpAdSignaldata.empty())
      sStartCmd += " '" + sCwd + "/ADAS/signaldata.csv'";
    if(!g_MpAdStopline.empty())
      sStartCmd += " '" + sCwd + "/ADAS/stopline.csv'";
    if(!g_MpAdStreetlight.empty())
      sStartCmd += " '" + sCwd + "/ADAS/streetlight.csv'";
    if(!g_MpAdUtilitypole.empty())
      sStartCmd += " '" + sCwd + "/ADAS/utilitypole.csv'";
    if(!g_MpAdVector.empty())
      sStartCmd += " '" + sCwd + "/ADAS/vector.csv'";
    if(!g_MpAdWhiteline.empty())
      sStartCmd += " '" + sCwd + "/ADAS/whiteline.csv'";
    sStartCmd += " > /dev/null 2>&1 &";
    //std::cerr << "DEBUG:[main] sStartCmd=" << sStartCmd << std::endl;

    // 変換したADAS-MAPの配信
    int iret;
    if((iret = system(sStartCmd.c_str())))
      std::cerr << "WARNING:[main-13] system(sStartCmd), iret=" << iret << std::endl;
  }
#ifdef ENABLE_TIME_MEASURE
  time_t timePblsh = time(NULL);
#endif

  if(bErrorFlg)
  {
    // エラーログ出力ストリームのクローズ
    std::cerr.rdbuf(cerrSave);
    ofserr.close();
  }

  if(bDebugFlg)
  {
    // デバッグログ出力ストリームのクローズ
    ofsdbg.close();
  }

#if ENABLE_TIME_MEASURE
  std::cout << "DEBUG:[main] timeRead=" << timeRead - timeStart
            << ", timeTrans=" << timeTrans - timeRead
            << ", timeWrite=" << timeWrite - timeTrans
            << ", timePblsh=" << timePblsh - timeWrite
            << ", timeTotal=" << timePblsh - timeStart << std::endl;
#endif

  std::cout << std::endl << "Harmoware-DMI static-dynamicmap converter: END" << std::endl;
  return 0;
}
