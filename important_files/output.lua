wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("KukaKr16")
gripper = wc:findFrame("Tool")
bottle = wc:findFrame("Bottle")
table = wc:findFrame("Table")
function setQ(q)
	qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])
	device:setQ(qq,state)
	rws.getRobWorkStudio():setState(state)
	rw.sleep(0.05)
end
setQ({-3.142,-0.827,-3.002,-3.143,0.099,-1.573})
rw.gripFrame(bottle,gripper,state)
setQ({-3.142,-0.827,-3.002,-3.143,0.099,-1.573})
setQ({-3.11905,-0.807404,-2.99264,-3.18561,0.0847537,-1.65657})
setQ({-3.09851,-0.782743,-2.93854,-3.19699,0.0825601,-1.73343})
setQ({-3.07073,-0.782029,-2.91058,-3.25458,0.0924172,-1.80437})
setQ({-2.99551,-0.774968,-2.87536,-3.29021,0.0586138,-1.82964})
setQ({-3.00179,-0.769292,-2.80602,-3.28848,0.0888444,-1.89448})
setQ({-2.92258,-0.731741,-2.82302,-3.29081,0.125314,-1.92077})
setQ({-2.92955,-0.711555,-2.83715,-3.36787,0.0796476,-1.95711})
setQ({-2.89316,-0.69241,-2.86239,-3.37804,0.0754588,-2.04401})
setQ({-2.81471,-0.659643,-2.8362,-3.38299,0.052397,-2.08312})
setQ({-2.75097,-0.625349,-2.81181,-3.43242,0.0671472,-2.12192})
setQ({-2.67628,-0.621984,-2.78811,-3.3976,0.0552547,-2.17186})
setQ({-2.64982,-0.62475,-2.75615,-3.33555,0.064036,-2.10727})
setQ({-2.62314,-0.62754,-2.72392,-3.27295,0.072894,-2.04213})
setQ({-2.59646,-0.630331,-2.69169,-3.21035,0.081752,-1.97698})
setQ({-2.56977,-0.633121,-2.65945,-3.14775,0.09061,-1.91183})
setQ({-2.54309,-0.635912,-2.62722,-3.08516,0.099468,-1.84668})
setQ({-2.57768,-0.629678,-2.6989,-3.09514,0.0713459,-1.79437})
setQ({-2.54395,-0.630521,-2.67691,-3.02947,0.0721633,-1.73061})
setQ({-2.60798,-0.654842,-2.67155,-3.04531,0.0902778,-1.66204})
setQ({-2.56147,-0.648716,-2.64042,-2.99105,0.0926946,-1.59975})
setQ({-2.58442,-0.67701,-2.64426,-2.97622,0.157161,-1.53431})
setQ({-2.53086,-0.668342,-2.61595,-2.9264,0.162205,-1.4731})
setQ({-2.47731,-0.659673,-2.58764,-2.87657,0.167249,-1.41189})
setQ({-2.42375,-0.651004,-2.55932,-2.82675,0.172293,-1.35068})
setQ({-2.37019,-0.642336,-2.53101,-2.77692,0.177337,-1.28947})
setQ({-2.31663,-0.633667,-2.5027,-2.7271,0.182381,-1.22826})
setQ({-2.26307,-0.624999,-2.47439,-2.67727,0.187425,-1.16705})
setQ({-2.20951,-0.61633,-2.44608,-2.62745,0.192469,-1.10584})
setQ({-2.15595,-0.607662,-2.41776,-2.57762,0.197513,-1.04463})
setQ({-2.10239,-0.598993,-2.38945,-2.5278,0.202557,-0.983422})
setQ({-2.04883,-0.590325,-2.36114,-2.47797,0.207601,-0.922212})
setQ({-1.99527,-0.581656,-2.33283,-2.42815,0.212645,-0.861002})
setQ({-1.94171,-0.572988,-2.30451,-2.37832,0.217689,-0.799792})
setQ({-1.88815,-0.564319,-2.2762,-2.3285,0.222733,-0.738582})
setQ({-1.83459,-0.55565,-2.24789,-2.27867,0.227777,-0.677372})
setQ({-1.78103,-0.546982,-2.21958,-2.22885,0.232822,-0.616162})
setQ({-1.72747,-0.538313,-2.19126,-2.17902,0.237866,-0.554952})
setQ({-1.67391,-0.529645,-2.16295,-2.1292,0.24291,-0.493742})
setQ({-1.62036,-0.520976,-2.13464,-2.07937,0.247954,-0.432532})
setQ({-1.5668,-0.512308,-2.10633,-2.02955,0.252998,-0.371322})
setQ({-1.51324,-0.503639,-2.07802,-1.97972,0.258042,-0.310112})
setQ({-1.45968,-0.494971,-2.0497,-1.92989,0.263086,-0.248902})
setQ({-1.40612,-0.486302,-2.02139,-1.88007,0.26813,-0.187692})
setQ({-1.35256,-0.477633,-1.99308,-1.83024,0.273174,-0.126482})
setQ({-1.299,-0.468965,-1.96477,-1.78042,0.278218,-0.0652719})
setQ({-1.24544,-0.460296,-1.93645,-1.73059,0.283262,-0.00406188})
setQ({-1.19188,-0.451628,-1.90814,-1.68077,0.288306,0.0571481})
setQ({-1.13832,-0.442959,-1.87983,-1.63094,0.29335,0.118358})
setQ({-1.08476,-0.434291,-1.85152,-1.58112,0.298394,0.179568})
setQ({-1.0312,-0.425622,-1.8232,-1.53129,0.303438,0.240778})
setQ({-0.977643,-0.416954,-1.79489,-1.48147,0.308482,0.301988})
setQ({-0.924084,-0.408285,-1.76658,-1.43164,0.313526,0.363198})
setQ({-0.870524,-0.399617,-1.73827,-1.38182,0.31857,0.424408})
setQ({-0.816965,-0.390948,-1.70995,-1.33199,0.323614,0.485618})
setQ({-0.763406,-0.382279,-1.68164,-1.28217,0.328658,0.546828})
setQ({-0.709846,-0.373611,-1.65333,-1.23234,0.333702,0.608038})
setQ({-0.656287,-0.364942,-1.62502,-1.18252,0.338746,0.669248})
setQ({-0.602727,-0.356274,-1.59671,-1.13269,0.34379,0.730458})
setQ({-0.549168,-0.347605,-1.56839,-1.08287,0.348834,0.791668})
setQ({-0.495609,-0.338937,-1.54008,-1.03304,0.353879,0.852878})
setQ({-0.442049,-0.330268,-1.51177,-0.983216,0.358923,0.914088})
setQ({-0.38849,-0.3216,-1.48346,-0.933391,0.363967,0.975298})
setQ({-0.334931,-0.312931,-1.45514,-0.883565,0.369011,1.03651})
setQ({-0.281371,-0.304262,-1.42683,-0.83374,0.374055,1.09772})
setQ({-0.227812,-0.295594,-1.39852,-0.783915,0.379099,1.15893})
setQ({-0.174252,-0.286925,-1.37021,-0.73409,0.384143,1.22014})
setQ({-0.120693,-0.278257,-1.34189,-0.684264,0.389187,1.28135})
setQ({-0.0671337,-0.269588,-1.31358,-0.634439,0.394231,1.34256})
setQ({-0.0135744,-0.26092,-1.28527,-0.584614,0.399275,1.40377})
setQ({0.039985,-0.252251,-1.25696,-0.534789,0.404319,1.46498})
setQ({0.0935444,-0.243583,-1.22864,-0.484964,0.409363,1.52619})
setQ({0.147104,-0.234914,-1.20033,-0.435138,0.414407,1.5874})
setQ({0.200663,-0.226246,-1.17202,-0.385313,0.419451,1.64861})
setQ({0.254222,-0.217577,-1.14371,-0.335488,0.424495,1.70982})
setQ({0.307782,-0.208908,-1.1154,-0.285663,0.429539,1.77103})
setQ({0.361341,-0.20024,-1.08708,-0.235838,0.434583,1.83224})
setQ({0.414901,-0.191571,-1.05877,-0.186012,0.439627,1.89345})
setQ({0.46846,-0.182903,-1.03046,-0.136187,0.444671,1.95466})
setQ({0.522019,-0.174234,-1.00215,-0.086362,0.449715,2.01587})
setQ({0.575579,-0.165566,-0.973833,-0.0365368,0.454759,2.07708})
setQ({0.629138,-0.156897,-0.945521,0.0132885,0.459803,2.13829})
setQ({0.682697,-0.148229,-0.917209,0.0631137,0.464847,2.1995})
setQ({0.736257,-0.13956,-0.888896,0.112939,0.469891,2.26071})
setQ({0.738779,-0.150411,-0.857967,0.0911044,0.505833,2.34527})
setQ({0.735261,-0.144771,-0.857406,0.0553842,0.512167,2.43822})
setQ({0.789041,-0.135784,-0.829363,0.103506,0.518259,2.50057})
setQ({0.808309,-0.143256,-0.774177,0.0553832,0.481515,2.55406})
setQ({0.793863,-0.158338,-0.759205,-0.0301198,0.467349,2.59682})
setQ({0.795327,-0.170658,-0.729507,-0.0936715,0.447002,2.66399})
setQ({0.795421,-0.169448,-0.740807,-0.149413,0.438444,2.74578})
setQ({0.777916,-0.14934,-0.771359,-0.161843,0.485424,2.82321})
setQ({0.834106,-0.139668,-0.738578,-0.11978,0.491678,2.88538})
setQ({0.824282,-0.138211,-0.71269,-0.0702986,0.51003,2.96567})
setQ({0.813691,-0.139354,-0.66096,-0.128497,0.566565,2.99071})
setQ({0.810886,-0.153086,-0.614751,-0.0666865,0.578013,3.05168})
setQ({0.86005,-0.125543,-0.594696,-0.0392511,0.647135,3.08154})
setQ({0.835264,-0.123224,-0.5674,-0.0837752,0.665257,3.16106})
setQ({0.820033,-0.13173,-0.549447,-0.158635,0.668561,3.22237})
setQ({0.815292,-0.140265,-0.527719,-0.2198,0.691363,3.29428})
setQ({0.810992,-0.141121,-0.534809,-0.230372,0.686155,3.39324})
setQ({0.809771,-0.142877,-0.506375,-0.321613,0.662727,3.41093})
setQ({0.782769,-0.133954,-0.483394,-0.390746,0.648687,3.47165})
setQ({0.835318,-0.124624,-0.449168,-0.354496,0.656241,3.53954})
setQ({0.887867,-0.115293,-0.414942,-0.318246,0.663796,3.60743})
setQ({0.940415,-0.105963,-0.380716,-0.281997,0.67135,3.67532})
setQ({0.992964,-0.0966329,-0.346489,-0.245747,0.678904,3.74321})
setQ({1.04551,-0.0873026,-0.312263,-0.209497,0.686458,3.8111})
setQ({1.09806,-0.0779723,-0.278037,-0.173247,0.694012,3.87899})
setQ({1.15061,-0.0686421,-0.24381,-0.136998,0.701567,3.94688})
setQ({1.20316,-0.0593118,-0.209584,-0.100748,0.709121,4.01477})
setQ({1.25571,-0.0499816,-0.175358,-0.0644983,0.716675,4.08266})
setQ({1.30826,-0.0406513,-0.141131,-0.0282486,0.724229,4.15055})
setQ({1.36081,-0.031321,-0.106905,0.00800112,0.731783,4.21844})
setQ({1.41335,-0.0219908,-0.0726789,0.0442508,0.739337,4.28633})
setQ({1.4659,-0.0126605,-0.0384526,0.0805006,0.746892,4.35422})
setQ({1.51845,-0.00333026,-0.00422629,0.11675,0.754446,4.42211})
setQ({1.571,0.006,0.03,0.153,0.762,4.49})
rw.gripFrame(bottle,table,state)
setQ({1.51845,-0.00333026,-0.00422629,0.11675,0.754446,4.42211})
setQ({1.4659,-0.0126605,-0.0384526,0.0805006,0.746892,4.35422})
setQ({1.41335,-0.0219908,-0.0726789,0.0442508,0.739337,4.28633})
setQ({1.36081,-0.031321,-0.106905,0.00800112,0.731783,4.21844})
setQ({1.30826,-0.0406513,-0.141131,-0.0282486,0.724229,4.15055})