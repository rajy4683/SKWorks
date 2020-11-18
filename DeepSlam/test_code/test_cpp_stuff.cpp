#include <vector>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    std::vector<std::int64_t> TensorDims({1, 20});
    std::cout << "vector of size:"<<TensorDims.size() << TensorDims.at(0)<<std::endl;
    //std::size_t size = 1;
    for (auto& d : TensorDims)
      std::cout << "Values:" << d<< std::endl;
    

}
