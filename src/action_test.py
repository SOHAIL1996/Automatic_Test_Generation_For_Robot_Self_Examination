#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
---------------------------------------------------- 
Action Tests

The ...
----------------------------------------------------
Supervisor: Prof. Dr. Paul Ploger
            Prof. Dr. Nico Hochgeschwender
            Alex Mitrevski 

Author    : Salman Omar Sohail
----------------------------------------------------
Date: August 19, 2020
----------------------------------------------------
"""

import pytest
from hypothesis import given, settings, Verbosity, example
import hypothesis.strategies as st

#################################################################################################
# Strategies
# 'nothing','just', 'one_of','none','choices', 'streaming','booleans', 'integers',
# 'floats', 'complex_numbers', 'fractions','decimals','characters', 'text', 'from_regex',
# 'binary', 'uuids','tuples', 'lists', 'sets', 'frozensets', 'iterables','dictionaries',
# 'fixed_dictionaries','sampled_from', 'permutations','datetimes', 'dates', 'times', 
# 'timedeltas','builds','randoms', 'random_module','recursive', 'composite','shared', 
# 'runner', 'data','deferred','from_type', 'register_type_strategy', 'emails'
#################################################################################################               
# Input type
# check pre conditions
# Ensure property is satisfied
# Generalized property
#################################################################################################
def sum(num1, num2):
    """It returns sum of two numbers"""
    return num1 + num2

@given(st.integers(), st.integers())
def test_sum_basic(num1, num2):
    assert sum(num1, num2) == num1 + num2

@given(st.integers(), st.integers())
@example(1, 2)
def test_sum_basic_with_example(num1, num2):
    assert sum(num1, num2) == num1 + num2

@settings(verbosity=Verbosity.verbose)
@given(st.integers(), st.integers())
def test_sum_with_property(num1, num2):
    assert sum(num1, num2) == num1 + num2
    # Test Identity property
    assert sum(num1, 0) == num1
    # Test Commutative property
    assert sum(num1, num2) == sum(num2, num1)

#Marking this test as expected to fail
@pytest.mark.xfail
@given(st.integers(), st.integers())
def test_sum_with_shrinking_example(num1, num2):
    assert sum(num1, num2) == num1 + num2
    # Test Identity property
    assert sum(num1, 0) == num1
    # Test Commutative property
    assert sum(num1, num2) == sum(num2, num1)
    assert num1 <= 30
#################################################################################################