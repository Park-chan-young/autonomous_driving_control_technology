/**
 searching person who fail to finish maraton

 using undordered_map (key + value)
 **/

#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>

using namespace std;

string solution(vector<string> participant, vector<string> completion) {
    string answer = "";
    
    unordered_map<string, int> participant_map;
    
    for (const string& name : participant) {
        participant_map[name]++;
    }
    
    for (const string& name : completion) {
        participant_map[name]--;
    }
    
    for (const auto& pair : participant_map) {
        if (pair.second == 1) {
            answer = pair.first;
            break;
        }
    }
    
    return answer;
}

// 동명이인이 있는 경우에도 적용할 수 있도록 key의 중복 허용 x
// 비순차 컨테이너 사용 & 등장 횟수를 value로 관리해 판단이 빠르도록 코딩