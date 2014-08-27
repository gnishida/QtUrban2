#version 420
#extension GL_EXT_gpu_shader4 : enable

in vec3 outColor;
in vec3 outUV;
in vec3 origVertex;// L

in vec3 varyingNormal;
in vec4 varyingLightVertexPosition;//position respect the camera view

out vec4 outputF;

uniform int mode;
uniform int terrainMode;
uniform sampler2D tex0;
uniform sampler2DArray tex_3D;
//uniform sampler3D tex_3D;

uniform vec4 terrain_size;//remove if terrainMode=2 removed
uniform sampler2D terrain_tex;

uniform sampler2D shadowMap;
uniform int shadowState;
 
uniform vec3 lightDir;
uniform mat4 light_mvpMatrix;
uniform mat4 light_biasMatrix;	//transfrom to [0,1] tex coordinates

uniform float waterMove;

const float ambientColor=0.1;
const float diffuseColor=1.0;
const float specularColor=1.0;

vec3 terrainMode3Colors[] = vec3[5]( 
   vec3( 0xf0/255.0, 0xed/255.0, 0xe5/255.0 ), // 7 mid river
   vec3( 0xca/255.0 ,0xdf/255.0, 0xaa/255.0 ), // 8 green
   vec3( 0xdf/255.0, 0xd9/255.0, 0xc3/255.0 ), // 9 coast
   vec3( 0xe9/255.0, 0xe5/255.0, 0xdc/255.0 ), //10 flat
   vec3( 0x50/255.0, 0x44/255.0, 0x43/255.0 ) //11 mountain 504443
);

vec2 poissonDisk4[4] = vec2[](
  vec2( -0.94201624, -0.39906216 ),
  vec2( 0.94558609, -0.76890725 ),
  vec2( -0.094184101, -0.92938870 ),
  vec2( 0.34495938, 0.29387760 )
);

vec2 poissonDisk16[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

// Returns a random number based on a vec3 and an int.
float random(vec3 seed, int i){
	vec4 seed4 = vec4(seed,i);
	float dot_product = dot(seed4, vec4(12.9898,78.233,45.164,94.673));
	return fract(sin(dot_product) * 43758.5453);
}

float shadowCoef(){
	vec4 shadow_coord2=light_mvpMatrix*vec4(origVertex,1.0);
	vec3 ProjCoords = shadow_coord2.xyz / shadow_coord2.w;
    vec2 UVCoords;
    UVCoords.x = 0.5 * ProjCoords.x + 0.5;
    UVCoords.y = 0.5 * ProjCoords.y + 0.5;
    float z = 0.5 * ProjCoords.z + 0.5;
	
	/// D
	bool HDShadow=true;
	float visibility=1.0f;
	if(HDShadow==true){
		// HDShadow
		for (int i=0;i<8;i++){
			int index = int(16.0*random(origVertex.xyz, i))%16;
			if ( texture2D( shadowMap, UVCoords + poissonDisk16[index]/3500.0 ).z  <  z ){
				visibility-=0.1;
			}
		}
	}else{
		// Low Shadow
		for (int i=0;i<4;i++){
			int index = int(4.0*random(origVertex.xyz, i))%4;
			if ( texture2D( shadowMap, UVCoords + poissonDisk4[index]/3500.0 ).z  <  z ){
				visibility-=0.2;
			}
		}

	}
	return visibility;
}

float facade(vec2 coordFac,vec2 maxFac,int windNumber){
	float intVal=1.0f;
	
	// SIDES
	if(coordFac.x<2.5||coordFac.y<2.5){
		//return 0;
		intVal= smoothstep(1.5,2.5,min(coordFac.x,coordFac.y));
	}
	if(coordFac.x>maxFac.x-2.5||coordFac.y>maxFac.y-2.5){
		float coord=min(maxFac.x-coordFac.x,maxFac.y-coordFac.y);
		intVal= min(intVal,smoothstep(1.5,2.5,coord));
	}
	if(intVal==0)
		return 0;
	// WINDOWS
	int numSt=int(ceil(maxFac.y*0.26f));//3.0m
	int numWin=int(ceil(maxFac.x*0.26f));
	//if(int(numWin*coordFac.x/maxFac.x)%2==0)
	//	return 1.0;
	
	int currS=int((numSt*coordFac.y)/maxFac.y);
	if(currS==0||currS==numSt-1)//side windows
		return intVal;
	//float currS=(numSt*coordFac.y)/maxFac.y;
	//if(currS<2.0)//first floor
	//	return max(intVal-(2.0-currS),0);
	int currWind=int((numWin*coordFac.x)/maxFac.x);
	if(currWind==0||currWind==numWin-1)//side windows
		return intVal;
	
	return intVal*texture( tex_3D, vec3(vec2(numWin,numSt)*coordFac.xy/maxFac.xy,7.0) )[windNumber];//6 window
	//return intVal;

}//

void main(){



	outputF =vec4(outColor,1.0);

	// SHADOW: From light
	if(shadowState==2){// no tex/color/shadows
		return;
	}

	// COLOR
	/*if(mode==1){
		outputF =vec4(outColor,1.0);
	}*/

	// TEXTURE
	if((mode&0xFF)==2||(mode&0xFF)==4||(mode&0xFF)==6){// tex / water / model texture
		outputF = texture( tex0, outUV.rg );
	}
	
	//////////////
	// TERRAIN
	if((mode&0xFF)==3){
		///////////////////////////
		// 0 FLAT (Maps)
		if(terrainMode==0){
			float height=outColor.r;///COMPUTED IN VERTEX
			if(height<=15){//water
				if(height<=13){
					outputF=vec4(0xa0/255.0,0xc3/255.0,0xff/255.0,1.0);//water blue
				}else{
					outputF=mix(
						vec4(0x84/255.0,0xa9/255.0,0xe6/255.0,1.0),//water blue dark 92bbff (mine 84a9e6)
						vec4(0xa0/255.0,0xc3/255.0,0xff/255.0,1.0),//water blue
						(15.0f-height)*0.5f);
				}
			}else{
				//outputF=vec4(0xf0/255.0,0xed/255.0,0xe5/255.0,1.0);//grey brighter
				outputF=vec4(0xe9/255.0,0xe5/255.0,0xdc/255.0,1.0);//gray dark
			}
			return;
		}
		///////////////////////////
		// 2 FLAT (Content Design)
		if(terrainMode==2){
			outputF=vec4(0,1,1,1);//0xe9/255.0,0xe5/255.0,0xdc/255.0,1.0);//gray dark
			vec2 terrainTexCoord=vec2(
				int(origVertex.x-terrain_size.x)/terrain_size.z,
				int(origVertex.y-terrain_size.y)/terrain_size.w
				);
			float height=255.0f*texture(terrain_tex,terrainTexCoord.rg).r;

			if(height<=6){//water
				if(height<=0){
					outputF=vec4(0x84/255.0,0xa9/255.0,0xe6/255.0,1.0);//water blue dark 92bbff (mine 84a9e6)
				}else{
					outputF=vec4(0xa0/255.0,0xc3/255.0,0xff/255.0,1.0);//water blue
				}
			}else{
				outputF=vec4(0xe9/255.0,0xe5/255.0,0xdc/255.0,1.0);//gray dark
				int heighStep=int(height-7);//index 0-4 (0 mid river, 1 green, 2 coast, 3 flat, 4 mountain)
				if(heighStep<0)heighStep=0;
				if(heighStep>4)heighStep=4;
				outputF=vec4(terrainMode3Colors[heighStep],1.0);
			}
			return;
		}
		///////////////////////////
		// 1 MOUNTAIN
		if(terrainMode==1){
			vec4 terrainColor=vec4(0,0,0,1.0);
			float factor;
			const float maxHeight=9.0;//7=255*7 1500m (change in vertex shader as well) HERE IS 9 TO ALLOW LESS HIGH MOUNTAINS
			float height=100.0f*(origVertex.z/maxHeight)/255.0;//0-100
			height=clamp(height,0.0,99.999999);//0-99.99

			int texInd=int(height/25);
			float interpTex=mod(height,25.0);
			// texture
			outputF=mix(
				texture( tex_3D, vec3(outUV.rg,texInd) ),
				texture( tex_3D, vec3(outUV.rg,texInd+1) ),
				interpTex/25.0);
		}
	}

	//////////////
	// FACADE
	if((mode&0xFF)==0x08){
		outputF =vec4(1.0,1.0,1.0,1.0);
		vec2 coordFac=outColor.xy;
		vec2 maxFac=outUV.xy;
		float dist=2.0;
		if(coordFac.x<2.0||coordFac.y<2.0){
			dist=min(coordFac.x,coordFac.y);
		}
		if(coordFac.x>maxFac.x-2.0||coordFac.y>maxFac.y-2.0){
			dist=min(dist,min(maxFac.x-coordFac.x,maxFac.y-coordFac.y));
		}
		float t=smoothstep(1.5,2.0,dist);
		outputF =mix(texture( tex0, coordFac.xy*0.1 ),vec4(1,1,1,1),t);
		
		//float deriv=fwidth(coordFac);
		//float amt = smoothstep(1-deriv,.1+deriv,coordFac.x);
		//outputF =mix(vec4(0.0,0.0,0.0,1.0),vec4(1.0,1.0,1.0,1.0),amt);
		return;
	}

	//////////////
	// HATCH WATER
	if((mode&0xFF)==11){
		vec2 coordFac=outUV.xy;
		//vec2 coord0=coordFac+vec2(waterMove);
		vec2 coord0=(coordFac+vec2(waterMove*3))*0.33;
		vec2 coord1=(coordFac+vec2(-waterMove*4))*0.25;
		vec3 normalC=texture( tex_3D, vec3(coord0,6.0) ).xyz;//6 water normal
		vec3 normalC2=texture( tex_3D, vec3(coord1,6.0) ).xyz;//6 water normal

		normalC = normalC*2.0-vec3(1.0);
		normalC2 = normalC2*2.0-vec3(1.0);
		normalC = normalize((normalC+normalC2)*0.5);
	
		float intensity=1.0f;

		intensity=1-(0.95*max(0.0, dot(normalize(-lightDir), normalize(normalC)))+0.05);
		int darknessInt=int(99.999*(1.0-intensity));
		int texInd=darknessInt/25;
		int interpTex=darknessInt%25;
		outputF=mix(
			texture( tex_3D, vec3(coordFac.xy*0.1,texInd) ),
			texture( tex_3D, vec3(coordFac.xy*0.1,texInd+1) ),
			interpTex/25.0);
		return;
	}
	//////////////
	// FACADE ARRAY TEXTURE
	if((mode&0xFF)==9||(mode&0xFF)==10){
		float intensity=1.0f;

		vec2 coordFac=outColor.xy;
		vec2 maxFac=outUV.xy;
		if((mode&0xFF)==10){
			coordFac=outUV.xy;
			intensity=outColor.x;
		}
		
		if((mode&0x0200)==0x0200){
			intensity=1-(0.95*max(0.0, dot(normalize(-lightDir), normalize(varyingNormal)))+0.05);
			intensity=clamp(intensity,0,1);
		}
		if((mode&0xFF)==9){
			int windNumber=int(outColor.z);
			intensity*=facade(coordFac,maxFac,windNumber);
		}

		int darknessInt=int(99.999*(1.0-intensity));
		int texInd=darknessInt/25;
		int interpTex=darknessInt%25;
		outputF=mix(
			texture( tex_3D, vec3(coordFac.xy*0.1,texInd) ),
			texture( tex_3D, vec3(coordFac.xy*0.1,texInd+1) ),
			interpTex/25.0);
		
		if(texInd<2){
			float perl=texture( tex_3D, vec3(coordFac.xy/maxFac.xy,5.0) ).x;//5.0 perlin noise
			outputF=mix(
				outputF,
				vec4(1,1,1,1),
				perl*0.3*(4-texInd));
		}
		return;
	}


	// LIGHTING
	vec4 ambientIllumination=vec4(0.05,0.05,0.05,0.0);
	vec4 diffuseIllumination=vec4(0.95,0.95,0.95,1.0);
	if(((mode&0x0200)==0x0200)||((mode&0xFF)==0x03)){//terran also gets lighting
		vec3 normal = varyingNormal;
		if(((mode&0x0FF)==0x05)||((mode&0xFF)==0x06))//seems that it needs it
			normal=normalize(normal);
		ambientIllumination = ambientColor*vec4(1.0,1.0,1.0,1.0);
		diffuseIllumination = (diffuseColor*vec4(1.0,1.0,1.0,1.0)) * max(0.0, dot(-lightDir, normal));
		//outputF=(ambientIllumination+diffuseIllumination)*outputF;
	}

	// SHADOW Disable
	if(shadowState==0){// 0 SHADOW Disable
		outputF=(ambientIllumination+diffuseIllumination)*outputF;
		return;
	}

	// SHADOW Render normal with shadows
	if(shadowState==1){// 1 SHADOW Render Normal
		float shadow_coef=0.95;
		shadow_coef= shadowCoef();
		outputF=(ambientIllumination+(shadow_coef+0.05)*diffuseIllumination)*outputF;
		return;
	}

}//

