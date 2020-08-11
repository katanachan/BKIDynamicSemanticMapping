/**
 *  Description: Unit tests for flow3f
 *  @file t_flow3f.cpp
 *  @author Aishwarya Unnikrishnan
 *  @version 1.1 08/07/20
 **/

#include "flow3f.h"
#include "unit_test_framework.h"
using namespace semantic_bki;
TEST(function_call_operator){
    flow3f vector(1.0f, 2.0f, 3.0f, 0.05f, 0.06f, 0.08f);
    ASSERT_EQUAL(vector(0), 1.0);
    ASSERT_EQUAL(vector(1), 2.0);
    ASSERT_EQUAL(vector(2), 3.0);
    ASSERT_EQUAL(vector(3), 0.05);
    ASSERT_EQUAL(vector(4), 0.06);
    ASSERT_EQUAL(vector(5), 0.08);
    
}
TEST_MAIN()