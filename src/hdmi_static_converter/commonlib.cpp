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
*  commonlib.cpp
*  共通処理ライブラリ (標準C++で書くこと)
****************************************/

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <limits.h>

#include "commonlib.h"

/************************************************************
*  CSV文字列のトークン化
*
*    input: str - CSV文字列
*    output: nothing
*    return: トークンのvector配列
************************************************************/
std::vector<std::string> getCsvToken(std::string str)
{
  std::string token;
  std::string::size_type left, right;
  std::vector<std::string> result;

  // 末尾の制御文字を削除
  str.erase(str.find_last_not_of("\n\r") + 1);

  // トークンに分解
  std::istringstream stream(str);
  while(getline(stream, token, ','))
    result.push_back(token);

  return result;
}

/************************************************************
*  CSV文字列のトリム処理したトークン化
*
*    input: str - CSV文字列
*    output: nothing
*    return: トークンのvector配列
************************************************************/
std::vector<std::string> getCsvTrimToken(std::string str)
{
  std::string token;
  std::string::size_type left, right;
  std::vector<std::string> result;

  // 末尾の制御文字を削除
  str.erase(str.find_last_not_of(" \n\r\t") + 1);

  // トークンに分解
  std::istringstream stream(str);
  while(getline(stream, token, ','))
  {
    // 前後の空白やタブを除去
    left = token.find_first_not_of(" \t");
    if (left != std::string::npos)
    {
      right = token.find_last_not_of(" \t");
      result.push_back(token.substr(left, right - left + 1));
    }
    else
      result.push_back("");
  }

  return result;
}

/************************************************************
*  文字列のトリム処理
*
*    input: str - 文字列
*    output: nothing
*    return: トリム処理した文字列
************************************************************/
std::string trimString(std::string str)
{
  std::string::size_type left, right;

  // 前後の空白やタブを除去
  left = str.find_first_not_of(" \t");
  if(left != std::string::npos)
  {
    right = str.find_last_not_of(" \t");
    return str.substr(left, right - left + 1);
  }

  return "";
}

/************************************************************
*  整数文字列を整数値に変換
*
*    input: str - 整数文字列
*    output: nothing
*    return: 整数値
************************************************************/
int convStrToInt(std::string str)
{
  const char* p = str.c_str();
  char* end;
  errno = 0;

  long x = strtol(p, &end, 10);
  if (p == end)
    throw std::invalid_argument("convStrToInt");
  if (errno == ERANGE || x < INT_MIN || x > INT_MAX)
    throw std::out_of_range("convStrToInt");

  return static_cast<int>(x);
}

/************************************************************
*  実数文字列を実数値に変換
*
*    input: str - 実数文字列
*    output: nothing
*    return: 実数値
************************************************************/
double convStrToDbl(std::string str)
{
  const char* p = str.c_str();
  char* end;
  errno = 0;

  double x = strtod(p, &end);
  if (p == end)
    throw std::invalid_argument("convStrToDbl");
  if (errno == ERANGE)
    throw std::out_of_range("convStrToDbl");

  return x;
}


/************************************************************
*  D.MMSS文字列を角度値に変換
*
*    input: str - D.MMSS文字列
*    output: nothing
*    return: 角度値 [deg]
************************************************************/
double convDmsToDeg(std::string str)
{
  int sign = 1;
  double deg;

  // 正負符号チェック
  std::string tstr = str.substr(0, 1);
  if(tstr == "-")
  {
    sign = -1;
    str = str.substr(1);
  }
  else if(tstr == "+")
  {
    sign = 1;
    str = str.substr(1);
  }

  // 小数点位置の検索
  std::string::size_type ppos = str.find(".");
  if(ppos == std::string::npos)
  {
    std::cerr << "WARNING:[convDmsToDeg-01] missing period [" << str << "]" << std::endl;
    throw std::invalid_argument("convDmsToDeg");
  }

  try
  {
    deg = convStrToDbl((str.substr(0, ppos)))  // Dの値
        + convStrToDbl((str.substr(ppos + 1, 2))) / 60.0  // MMの値
        + convStrToDbl((str.substr(ppos + 3, 2))) / 3600.0  // SSの値
        + convStrToDbl(("0." + str.substr(ppos + 5))) / 3600.0;  // SS以下の値
  }
  catch(...)
  {
    std::cerr << "WARNING:[convDmsToDeg-02] illegal value [" << str << "]" << std::endl;
    throw std::invalid_argument("convDmsToDeg");
  }
  deg *= sign;

  return deg;
}

/************************************************************
*  角度値をD.MMSS文字列に変換
*
*    input: deg  - 角度値 [deg]
*           pflg - 出力精度フラグ (true=SS以下4桁、false=SSまで)
*    output: nothing
*    return: D.MMSS文字列
************************************************************/
std::string convDegToDms(double deg, bool pflg)
{
  std::ostringstream ostr;
  int i;

  if((deg >= 360.0) || (deg <= -360.0))
  {
    std::cerr << "WARNING:[convDegToDms-01] illegal deg=" << deg << std::endl;
    throw std::out_of_range("convDegToDms");
  }

  if(deg < 0)
  {
    ostr << "-";  // 符号の値
    deg = -deg;
  }

  // D.の値
  i = int(deg);
  ostr << i << ".";

  // MMの値
  deg = (deg - i) * 60;
  i = int(deg);
  ostr << std::setw(2) << std::setfill('0') << i;

  if(pflg)
  {
    // SSの値
    deg = (deg - i) * 60;
    i = int(deg);
    ostr << std::setw(2) << std::setfill('0') << i;

    // SS以下4桁の値
    deg = (deg - i) * 10000;
    i = int(deg);
    ostr << std::setw(4) << std::setfill('0') << i;
  }
  else
  {
    // SSの値
    deg = (deg - i) * 60;
    i = int(deg);
    ostr << std::setw(2) << std::setfill('0') << i;
  }

  return ostr.str();
}
