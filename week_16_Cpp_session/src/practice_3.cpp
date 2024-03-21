/**
 Scoville

 using priority queue & pop method
 **/

#include <vector>
#include <algorithm>
#include <queue>

using namespace std;

int solution(std::vector<int> scoville, int K) {
    int answer = 0;
    priority_queue<int, vector<int>, greater<int>> scoville_pq(scoville.begin(), scoville.end());
    
    while (scoville_pq.top() < K) {
        if (scoville_pq.size() == 1) {
            return -1;
        }
        
        int mildest_scoville = scoville_pq.top();
        scoville_pq.pop();
        int second_mildest_scoville = scoville_pq.top();
        scoville_pq.pop();
        int mixed_scoville = mildest_scoville + second_mildest_scoville * 2;
        scoville_pq.push(mixed_scoville);
        answer += 1;
    }

    return answer;
}