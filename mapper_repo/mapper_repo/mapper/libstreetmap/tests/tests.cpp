#include <algorithm>
#include <set>
#include <vector>
#include <string>
#include <unittest++/UnitTest++.h>
#include "m1.h"
#include "/cad2/ece297s/public/m1/tests/unit_test_util.h"
#include "StreetsDatabaseAPI.h"

using ece297test::relative_error;

extern std::vector<std::string> streetNames;
extern unsigned numStreet;

struct MapFixture {
    MapFixture() {
        //Load the map
        load_map("/cad2/ece297s/public/maps/toronto_canada.streets.bin");

	rng = std::minstd_rand(3);
	rand_street = std::uniform_int_distribution<unsigned>(0, getNumberOfStreets()-1);

        for(size_t i = 0; i < 500000; i++){
	  street_id.push_back(rand_street(rng));
	}

	for(size_t i = 0; i < 500000; i++){
	  street_names.push_back(streetNames[i]);
	}
	
	for(size_t i = 0; i < 500000; i++){
          for(size_t j = 0; j < 500000; j++){
            if(streetNames[j] == street_names[j]){
	      street_ids[i].push_back(j); 
	    }
	  }
	}
	
	
    }

    ~MapFixture() {
        //Clean-up
        close_map();
    }

  std::minstd_rand rng;
  std::uniform_int_distribution<unsigned>rand_street;
  std::vector<unsigned>load_street_id;
  std::vector<std::vector<unsigned>>street_ids(numStreet);
  std::vector<std::string>street_names;
  
};

SUITE(test_toronto_canada) {
    TEST_FIXTURE(MapFixture, street_ids_from_street_name) {
      std :: vector<unsigned> expected;
      std :: vector<unsigned> actual;

      expected = street_ids[0];
      actual = find_street_ids_from_name(street_names[0]);
      std::sort(actual.begin(), actual.end());
      CHECK_EQUAL(expected, actual);
      
      {

	ECE297_TIME_CONSTRAINT(965);
	std::vector<unsigned> result;
	for(size_t i = 0; i < 500000; i++){
	  result = find_street_ids_from_name(street_names[i]);
	}
	
      }
    }
}
