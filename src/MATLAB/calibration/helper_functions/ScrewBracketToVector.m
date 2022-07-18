function S=ScrewBracketToVector(S_bracket)

w = SkewMatrixToVector(S_bracket(1:3,1:3));
v = S_bracket(1:3,4);

S = [w;v];

end