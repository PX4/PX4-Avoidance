#include <gtest/gtest.h>

#include <avoidance/usm.h>

#include <iostream>

using namespace usm;

/*
* Copyright (c) 2019 Julian Kent. All rights reserved.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of libgnc nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*/

/*
 * Implement a basic state machine with 2 paths (A & B), and error state,
 * and a transition table. Use this as an example for your use case!
 */

enum TestStates { START, PATH_A_1, PATH_A_2, END, PATH_B_1, PATH_B_2, PATH_B_3, CLEANUP };

class TestStateMachine final : public StateMachine<TestStates> {
 public:
  TestStateMachine() : StateMachine<TestStates>(START) {}

  bool path_a = true;
  bool trigger_error_once = false;

  bool start_called = false;
  bool a1_called = false;
  bool a2_called = false;
  bool b1_called = false;
  bool b2_called = false;
  bool b3_called = false;
  bool end_called = false;
  bool cleanup_called = false;

 protected:
  Transition runCurrentState() override {
    if (trigger_error_once) {
      trigger_error_once = false;
      return Transition::ERROR;
    }

    // clang-format off
    switch(getState()) {
        case START: return start();
        case PATH_A_1: return a1();
        case PATH_A_2: return a2();
        case END: return end();
        case PATH_B_1: return b1();
        case PATH_B_2: return b2();
        case PATH_B_3: return b3();
        case CLEANUP: return cleanup();
    }
    // clang-format on
  }

  TestStates chooseNextState(TestStates currentState, Transition transition) override {
    // clang-format off
    USM_TABLE(currentState, CLEANUP,
      USM_STATE(transition, START,    USM_MAP(Transition::NEXT1, PATH_A_1);
                                      USM_MAP(Transition::NEXT2, PATH_B_1));
      USM_STATE(transition, PATH_A_1, USM_MAP(Transition::NEXT1, PATH_A_2);
                                      USM_MAP(Transition::ERROR, PATH_B_3));
      USM_STATE(transition, PATH_A_2, USM_MAP(Transition::NEXT1, END));
      USM_STATE(transition, PATH_B_1, USM_MAP(Transition::NEXT1, PATH_B_2));
      USM_STATE(transition, PATH_B_2, USM_MAP(Transition::NEXT1, PATH_B_3));
      USM_STATE(transition, PATH_B_3, USM_MAP(Transition::NEXT1, END));
      USM_STATE(transition, CLEANUP,  USM_MAP(Transition::NEXT1, END))
    );
    // clang-format on
  }

 private:
  Transition start() {
    start_called = true;
    return path_a ? Transition::NEXT1 : Transition::NEXT2;
  }
  Transition a1() {
    a1_called = true;
    return Transition::NEXT1;
  }
  Transition a2() {
    a2_called = true;
    return Transition::NEXT1;
  }
  Transition b1() {
    b1_called = true;
    return Transition::NEXT1;
  }
  Transition b2() {
    b2_called = true;
    return Transition::NEXT1;
  }
  Transition b3() {
    b3_called = true;
    return Transition::NEXT1;
  }
  Transition end() {
    end_called = true;
    return Transition::REPEAT;
  }
  Transition cleanup() {
    cleanup_called = true;
    return Transition::NEXT1;
  }
};

TEST(StateMachine, runPathA) {
  TestStateMachine m;
  ASSERT_EQ(m.getState(), START);

  ASSERT_FALSE(m.start_called);
  m.iterateOnce();
  ASSERT_TRUE(m.start_called);
  ASSERT_EQ(m.getState(), PATH_A_1);

  ASSERT_FALSE(m.a1_called);
  m.iterateOnce();
  ASSERT_TRUE(m.a1_called);
  ASSERT_EQ(m.getState(), PATH_A_2);

  ASSERT_FALSE(m.a2_called);
  m.iterateOnce();
  ASSERT_TRUE(m.a2_called);
  ASSERT_EQ(m.getState(), END);

  ASSERT_FALSE(m.end_called);
  m.iterateOnce();
  ASSERT_TRUE(m.end_called);
}

TEST(StateMachine, runPathB) {
  TestStateMachine m;
  m.path_a = false;
  ASSERT_EQ(m.getState(), START);

  ASSERT_FALSE(m.start_called);
  m.iterateOnce();
  ASSERT_TRUE(m.start_called);
  ASSERT_EQ(m.getState(), PATH_B_1);

  ASSERT_FALSE(m.b1_called);
  m.iterateOnce();
  ASSERT_TRUE(m.b1_called);
  ASSERT_EQ(m.getState(), PATH_B_2);

  ASSERT_FALSE(m.b2_called);
  m.iterateOnce();
  ASSERT_TRUE(m.b2_called);
  ASSERT_EQ(m.getState(), PATH_B_3);

  ASSERT_FALSE(m.b3_called);
  m.iterateOnce();
  ASSERT_TRUE(m.b3_called);
  ASSERT_EQ(m.getState(), END);

  ASSERT_FALSE(m.end_called);
  m.iterateOnce();
  ASSERT_TRUE(m.end_called);
}

TEST(StateMachine, enterDefaultError) {
  TestStateMachine m;
  ASSERT_EQ(m.getState(), START);

  ASSERT_FALSE(m.start_called);
  m.iterateOnce();
  ASSERT_TRUE(m.start_called);
  ASSERT_EQ(m.getState(), PATH_A_1);

  ASSERT_FALSE(m.a1_called);
  m.iterateOnce();
  ASSERT_TRUE(m.a1_called);
  ASSERT_EQ(m.getState(), PATH_A_2);

  m.trigger_error_once = true;
  m.iterateOnce();
  ASSERT_EQ(m.getState(), CLEANUP);

  ASSERT_FALSE(m.cleanup_called);
  m.iterateOnce();
  ASSERT_TRUE(m.cleanup_called);
  ASSERT_EQ(m.getState(), END);

  ASSERT_FALSE(m.end_called);
  m.iterateOnce();
  ASSERT_TRUE(m.end_called);
}

TEST(StateMachine, enterCustomError) {
  TestStateMachine m;
  ASSERT_EQ(m.getState(), START);

  ASSERT_FALSE(m.start_called);
  m.iterateOnce();
  ASSERT_TRUE(m.start_called);
  ASSERT_EQ(m.getState(), PATH_A_1);

  m.trigger_error_once = true;
  m.iterateOnce();
  ASSERT_EQ(m.getState(), PATH_B_3);

  ASSERT_FALSE(m.b3_called);
  m.iterateOnce();
  ASSERT_TRUE(m.b3_called);
  ASSERT_EQ(m.getState(), END);

  ASSERT_FALSE(m.end_called);
  m.iterateOnce();
  ASSERT_TRUE(m.end_called);
  ASSERT_EQ(m.getState(), END);
}
