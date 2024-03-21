/**
 Kth number

 using vector & sort method
 **/

#include <vector>
#include <iostream>
#include <algorithm>

using namespace std;

vector<int> solution(vector<int> array, vector<vector<int>> commands) {
    vector<int> answer;
    
    for (const auto& cmd : commands) {
        int i = cmd[0];
        int j = cmd[1];
        int k = cmd[2];

        vector<int> trim(array.begin() + i - 1, array.begin() + j);

        sort(trim.begin(), trim.end());

        answer.push_back(trim[k-1]);
    }
    
    return answer;
}
