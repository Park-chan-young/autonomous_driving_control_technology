/**
 rotating queue

 using deque & pop_front method
 **/

#include <iostream>
#include <deque>

int main() {
	int queue_size;
	int num_of_extract;
	int index;
	int target_index;
	int number_of_movement = 0;
    std::deque<int> que;

	std::cin >> queue_size >> num_of_extract;

	for (int i=1; i <= queue_size; ++i)
	    que.push_back(i);

	while (num_of_extract--){
		std::cin >> target_index;

		for(int k=0; k < que.size(); ++k){
			if (que[k] == target_index){
				index = k;
				break;
			}
		}

		if(index < (que.size() - index)){
			while(1){
				if(que.front() == target_index){
					que.pop_front();
					break;
				}
				++number_of_movement;
				que.push_back(que.front());
				que.pop_front();
			}
		}
		else{
			while(1){
				if(que.front() == target_index){
					que.pop_front();
					break;
				}
				++number_of_movement;
				que.push_front(que.back());
				que.pop_back();
			}
		}
	}
	std::cout << number_of_movement;

	return 0;
}


// #include <iostream>
// #include <deque>

// int receive_input(){
// 	int N, M;
// 	std::cin >> N >> M;
// 	return N, M;
// }

// std::pair<int, std::deque<int>> move_to_left(int l_target_index, std::deque<int> l_que){
//     int l_number_of_movement;
// 	while(1){
// 		if(l_que.front() == l_target_index){
// 			l_que.pop_front();
// 			break;
// 		}
// 		++l_number_of_movement;
// 		l_que.push_back(l_que.front());
// 		l_que.pop_front();
// 	}
// 	return std::make_pair(l_number_of_movement, l_que);
// }

// std::pair<int, std::deque<int>> move_to_right(int r_target_index, std::deque<int> r_que){
//     int r_number_of_movement;
// 	while(1){
// 		if(r_que.front() == r_target_index){
// 			r_que.pop_front();
// 			break;
// 		}
// 		++r_number_of_movement;
// 		r_que.push_front(r_que.back());
// 		r_que.pop_back();
// 	}
// 	return std::make_pair(r_number_of_movement, r_que);
// }

// int main() {
	
// 	int queue_size, num_of_extract = receive_input();

// 	int index;
// 	int num_of_move_to_left = 0;
// 	int num_of_move_to_right = 0;
	
// 	std::deque<int> que;
// 	for (int i=1; i <= queue_size; ++i)
// 	    que.push_back(i);

// 	for (int j=0; j<num_of_extract; j++){
// 		int target_index;
// 		std::cin >> target_index;
// 		for(int k=0; k < queue_size; k++){
// 			if (que[k] == target_index){
// 				index = k;
// 				break;
// 			}
// 		}
// 		if(index <= queue_size/2){
// 			int num_of_move_to_left = move_to_left(target_index, que).first;
// 			que = move_to_left(target_index, que).second;
// 		}
// 		else{
// 			int num_of_move_to_right = move_to_right(target_index, que).first;
// 			que = move_to_right(target_index, que).second;
// 		}
// 	}
//     int result = num_of_move_to_left + num_of_move_to_right;
// 	std::cout << result;

// 	return 0;

// }
