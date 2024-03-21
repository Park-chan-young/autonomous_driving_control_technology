#include <iostream>
#include <list>
#include <unordered_map>

using namespace std;

int main() {
    int K, L;
    cin >> K >> L;

    list<string> students_ids;
    unordered_map<string, list<string>::iterator> unique_ids;

    for (int i = 0; i < L; ++i) {
        string student_id;
        cin >> student_id;

        // if cannot find iterator of student_id, same as .end() iterator
        auto iter = unique_ids.find(student_id);
        if (iter != unique_ids.end()) {
            students_ids.erase(iter->second);
        }
        students_ids.push_back(student_id);
        unique_ids[student_id] = --students_ids.end();
    }

    int count = 0;
    for (const auto& id : students_ids) {
        cout << id << endl;
        if (++count == K) {
            break;
        }
    }

    return 0;
}