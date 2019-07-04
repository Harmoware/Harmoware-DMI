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
#ifndef _COMMONLIB_H_
#define _COMMONLIB_H_

/************************************************************
*  グローバル変数定義
************************************************************/

/************************************************************
*  グローバル関数定義
************************************************************/
extern std::vector<std::string> getCsvToken(std::string str);
extern std::vector<std::string> getCsvTrimToken(std::string str);
extern std::string trimString(std::string str);
extern double convDmsToDeg(std::string str);
extern std::string convDegToDms(double deg, bool pflg);
extern int convStrToInt(std::string str);
extern double convStrToDbl(std::string str);

#endif // _COMMONLIB_H_
