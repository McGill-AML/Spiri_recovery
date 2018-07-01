px4_add_library(eFLL
	Fuzzy.cpp
	FuzzyComposition.cpp
	FuzzyInput.cpp
	FuzzyIO.cpp
	FuzzyOutput.cpp
	FuzzyRule.cpp
	FuzzyRuleAntecedent.cpp
	FuzzyRuleConsequent.cpp
	FuzzySet.cpp
)

target_include_directories(eFLL PRIVATE ${PX4_BINARY_DIR}/src/lib/eFLL)
