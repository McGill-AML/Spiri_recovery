px4_add_module(
	MODULE modules__impact_characterization
	MAIN impact_characterization
	STACK_MAIN 2000
	COMPILE_FLAGS -Os
	SRCS
		initialize_fuzzylogicprocess.cpp
		impact_characterization.cpp
		Fuzzy.h
		FuzzySet.h
		FuzzyInput.h
		FuzzyIO.h
		FuzzyOutput.h
		FuzzyRuleConsequent.h
		FuzzyRuleAntecedent.h
		
	DEPENDS
		platforms__common
	)
