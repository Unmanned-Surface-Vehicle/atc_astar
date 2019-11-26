#ifndef POSI_HPP
#define POSI_HPP

#include<unordered_map>

struct Pos{

	int x;
	int y;
	int th;
};

// Comparison method for Pos struct
inline bool operator==(const Pos& pos1, const Pos& pos2){
		return pos1.x == pos2.x && pos1.y == pos2.y;
}

// Comparison method for Pos struct
inline bool operator!=(const Pos& pos1, const Pos& pos2){
		return pos1.x != pos2.x || pos1.y != pos2.y;
}

// Comparison method for Pos struct
inline bool operator<(const Pos& pos1, const Pos& pos2){
		return pos1.x < pos2.x || pos1.y < pos2.y;
}

// Hash definition for specialized <unordered_set> of Pos struct
namespace std{

	template<> struct hash<Pos>{
        typedef Pos argument_type;
        typedef std::size_t result_type;
		std::size_t operator()(const Pos& p) const noexcept
		{
		    return std::hash<int>()(p.x ^ (p.y << 4));
		}
	};
}

#endif