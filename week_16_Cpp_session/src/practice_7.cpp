#include<iostream>
#include<vector>
using namespace std;

int main() {
    int N;
    cin >> N;

    vector<int> A(N);

    for (int i=0; i<N; ++i){
        cin >> A[i];
    }

    int M;
    cin >> M;

    for (int i=0; i<M; ++i) {
        int target;
        cin >> target;
        int count = 0;

        for (int j=0; j<N; ++j) {
            if (A[j] == target) {
                count++;
            }
        }
        cout << count << " ";
    }
}