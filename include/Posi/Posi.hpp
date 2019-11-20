#ifndef POSI_HPP
#define POSI_HPP

#include<unordered_map>

struct Posi{

	int x;
	int y;
};

// Comparison method for Posi struct
inline bool operator==(const Posi& pos1, const Posi& pos2){
		return pos1.x == pos2.x && pos1.y == pos2.y;
}

// Comparison method for Posi struct
inline bool operator!=(const Posi& pos1, const Posi& pos2){
		return pos1.x != pos2.x || pos1.y != pos2.y;
}

// Comparison method for Posi struct
inline bool operator<(const Posi& pos1, const Posi& pos2){
		return pos1.x < pos2.x || pos1.y < pos2.y;
}

// Hash definition for specialized <unordered_set> of Posi struct
namespace std{

	template<> struct hash<Posi>{
        typedef Posi argument_type;
        typedef std::size_t result_type;
		std::size_t operator()(const Posi& p) const noexcept
		{
		    return std::hash<int>()(p.x ^ (p.y << 4));
		}
	};
}

#endif