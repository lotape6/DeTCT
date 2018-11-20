#include <string>
#include <iostream>

std::string ColorCluster::classsifyColor(Color color){

  if(color.getV() <= 64) return "BLACK";

  if(color.getS() <= 64){
    if(color.getV() > 200) return "WHITE";
    else return "GRAY";
  }

  if(color.getH() < 15) return "RED";

  if(color.getH() < 45) return "YELLOW";

  if(color.getH() < 75) return "GREEN";

  if(color.getH() < 105) return "CYAN";

  if(color.getH() < 135) return "BLUE";

  if(color.getH() < 165) return "MAGENTA";

  return "UNDEFINED";

}
