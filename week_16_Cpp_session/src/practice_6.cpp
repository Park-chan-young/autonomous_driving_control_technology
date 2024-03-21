#include<iostream>
#include<vector>
#include<algorithm>
using namespace std;

int main() {
    int N;
    cin >> N;
    vector<int> A(N);
    for (int i=0; i<N; ++i){
        cin >> A[i];
    }
    sort(A.begin(), A.end());
    int M, temp;
    cin >> M;
    for (int i=0; i<N; ++i) {
        cin >> temp;
        cout << binary_search(A.begin(), A.end(), temp) << '\n';
    }
}