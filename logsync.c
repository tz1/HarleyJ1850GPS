#include <stdio.h>
#include <string.h>

char syncbuf[1024][256];

int main( int argc, char *argv[] ) {
    char buf[256];
    int cnt, mstik, newtik, i, jcnt, gcnt, jtik, gtik, icnt;
    icnt = 0;
    while( !feof(stdin) ) {
	memset( buf, 0, sizeof(buf) );
	gets(buf);
	printf( "%06d%s\r\n", icnt++, buf );
	if( buf[3] == '=' )
	    break;
    }
    cnt = 0;
    //    strcpy( syncbuf[cnt++], buf );
    while( !feof(stdin) ) {
	memset( buf, 0, sizeof(buf) );
	gets(buf);
	if( buf[3] == 'J' ) {
	    strcpy(syncbuf[cnt++], buf );
	    continue;
	}
	printf( "%06d%s\r\n", icnt++, buf );
	if( buf[3] == '$' )
	    break;
    }
    mstik = atoi(buf);
    while( !feof(stdin) ) {
	memset( buf, 0, sizeof(buf) );
	gets(buf);
	if( buf[3] == 'J' ) {
	    strcpy(syncbuf[cnt++], buf );
	    continue;
	}
	newtik = atoi(buf);
	if(newtik < mstik )
	    break;
	mstik = newtik;
	printf( "%06d%s\r\n", icnt++, buf );
    }
    gtik = newtik;
    jcnt = 100000;
    gcnt = 100000;
    icnt = 100000;
    jtik = 0;
    for( i = 0 ; i < cnt; i++ ) {
	jtik = atoi(syncbuf[i]);
	printf( "%06d%s\r\n", jcnt, syncbuf[i] );
    }
    i = 0;
    while( !feof(stdin) ) {
	memset( buf, 0, sizeof(buf) );
	gets(buf);
	switch( buf[3] ) {
	case '$':
	    newtik = atoi(buf);
	    if( newtik < gtik )
		gcnt++;
	    gtik = newtik;
	    if( !strncmp( "$GPGSA,", &buf[3], 7  ) )
		icnt = gcnt; // PPS follows this as it is one-per and after the RMC
	    printf( "%06d%s\r\n", gcnt, buf );
	    break;
	case '=':
	    if( icnt < jcnt )
		icnt = jcnt;
	    printf( "%06d%s\r\n", ++icnt, buf );
	    break;
	case 'J':
	    newtik = atoi(buf);
	    if( newtik < jtik )
		jcnt++;
	    jtik = newtik;
	    printf( "%06d%s\r\n", jcnt, buf );
	    if( jcnt < icnt ) {
		jcnt = icnt;
		jtik = 0;
	    }
	    break;
        default:
	    printf( "???%06d%s", jcnt, buf );
	    break;
	}
    }

}
