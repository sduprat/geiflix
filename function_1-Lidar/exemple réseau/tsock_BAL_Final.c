/* librairie standard ... */
#include <stdlib.h>
/* pour getopt */
#include <unistd.h>
/* déclaration des types de base */
#include <sys/types.h>
/* constantes relatives aux domaines, types et protocoles */
#include <sys/socket.h>
/* constantes et structures propres au domaine UNIX */
#include <sys/un.h>
/* constantes et structures propres au domaine INTERNET */
#include <netinet/in.h>
/* structures retournées par les fonctions de gestion de la base de données du réseau */
#include <netdb.h>
/* pour les entrées/sorties */
#include <stdio.h>
/* pour la gestion des erreurs */
#include <errno.h>
/* pour la gestion des chaines de caracteres */
#include <string.h>

#include <sys/time.h>


#define BUFSIZE 1024

#define TABSIZE_MSG_BUF 40 ///Nombre caractere max possible pour chaque msg


typedef struct Message Message; ///Structure stockages des messages
struct Message
{
    Message *suivant;
    char msg[TABSIZE_MSG_BUF];

};

typedef struct Lettre Lettre; ///Structure stockages des Lettres qui vont contenir les messages
struct Lettre
{
    Lettre *suivant;
    Message *premiermsg;
    int num; ///numéro de l'envoyeur

};

typedef struct ListeBal ListeBal; ///Structure de réference pour avoir un point de repere sur la premiere lettre
struct ListeBal
{
    Lettre *premier;
};

int num_envoi_existant(ListeBal *liste,int num_envoi);

ListeBal *init() ///initialise la liste , je met un coureur de base dedans
{
    ListeBal *liste =malloc(sizeof(*liste));//Tu crees une liste et tu lui alloue de la mémoire , il faut un pointeur de liste car il faudra la modifier dans des fonctions
    Lettre *let=calloc(1,sizeof(*let)); //Tu crées un element et tu lui alloue de la mémoire
    let->num=1; //init du num (Point faible du programme car on doit creer forcément une lettre alors que le num ne sera pas forcément utilsé)
	let->premiermsg=NULL;//Pas de premier message init a NULL
	let->suivant=NULL; //Pas d'autre lettre init a NULL
    liste->premier=let; //Tu donnes l'adresse de l'element a ton liste , il sera donc le premier
    return liste;//retour adresse de liste
}


void AddLettreFin(ListeBal *liste,int num_envoi) ///Ajoute une lettre pour stocker les messages du num_envoi si elle n'existe pas déja
{
	if(num_envoi_existant(liste,num_envoi)==0) //permet de voir si lettre existe ou non
	{
		Lettre *NvEle=malloc(sizeof(*NvEle)); //Création nouvelle lettre
		NvEle->num=num_envoi; //init du num de la nouvelle lettre
		Lettre *Parcours=liste->premier; //init du parcours au début des lettres
		while(Parcours->suivant!=NULL) //on parcours jusqu'au dernier élement de la liste non null
		{
			Parcours=Parcours->suivant;
		}
		//Parcours pointe sur le dernier élement non null de la liste
		NvEle->suivant=NULL; //on met le suivant de la nouvelle lettre a NULL
		NvEle->premiermsg=NULL;//Pareil pour son premier messages
		Parcours->suivant=NvEle;//Ajout de la lettre a la fin
	}
}


void EffacerMessage(ListeBal *liste,int num_envoi)///Effaces TOUS les messages d'une lettre MAIS pas la lettre en elle meme (Lettre devient alors vide)
{
    Lettre *Parcours=liste->premier;
	if(num_envoi_existant(liste,num_envoi)==0)//Si la lettre n'existe pas
	{
		printf("Aucun Message à effacer dans le recepteur n°%d car lettre non-existante \n",num_envoi);
	}
    while(Parcours!=NULL) //on va jusqu'a la fin de la liste (si on avait mis Parcours->suivant!=NULL on aurait pas traité le tout dernier élement car on serait sortie du while direct )
    {
		if(Parcours->num==num_envoi) //si le parcours pointe sur la lettre dont on souhaite effacer les messages
			{
				if(Parcours->premiermsg==NULL) //si la lettre n'a pas de premier message rien a effacer
				{
					printf("Aucun Message à effacer dans le recepteur n°%d \n",num_envoi);
				}
				else
				{
					Message *CurrentMsg=Parcours->premiermsg;
					Message *NextMsg=Parcours->premiermsg;

					while(CurrentMsg!=NULL) //Effacement des messages du premier jusqu'au dernier(début->fin)
					{
						NextMsg=CurrentMsg->suivant;
						free(CurrentMsg);
						CurrentMsg=NextMsg;
					}
					Parcours->premiermsg=NULL; //reinit de premier msg a NULL
				}
			}
		Parcours=Parcours->suivant;
	}
}


void LectureMessage(ListeBal *liste,int num_envoi)///Fonction permettant de lire tous les messages d'une lettre (Utilisé pour faire des TEST)
{
    Lettre *Parcours=liste->premier;
    while(Parcours!=NULL)
    {
		if(Parcours->num==num_envoi)
			{

				if(Parcours->premiermsg==NULL)
				{
					printf("Aucun Message pour le recepteur n°%d \n",num_envoi);
				}
				else
				{


					Message *ParcoursMsg=Parcours->premiermsg;
		            int i=1;
					while(ParcoursMsg!=NULL)
					{
					    printf("Message n°%d: \n",i);
					    printf("%s \n",ParcoursMsg->msg);
					    i++;
						ParcoursMsg=ParcoursMsg->suivant;
					}
					 printf("Fin Lecture \n");

				}
			}
		Parcours=Parcours->suivant;
	}

}


int num_envoi_existant(ListeBal *liste,int num_envoi) ///Fonction qui renvoi 1 si la lettre ayant num_envoi pour ID existe déja , 0 dans le cas contraire
{
    Lettre *Parcours=liste->premier;
	int existant=0;
    while(Parcours!=NULL) //Parcours de toute les lettres
    {
		if(Parcours->num==num_envoi)//Si une lettre a déja pour ID num_envoi on retourne existant=1
			{
				existant=1;
				break;
			}
        Parcours=Parcours->suivant;
    }
    return existant;
}


int retourmsg(char*buf,ListeBal *liste,int num_envoi,int num_msg)
///Fonction avec deux objectif :
///					-Retourner le message numéro "num_msg" d'une lettre "num_envoi" dans buf ex:On retourne le premier message de la Lettre 2
///					-Renvoyer 0 si il n'y a aucun message dans la lettre ou alors que le numéro du message est inexistant ex: on veut le message numéro 10 de la Lettre3 alors qu'elle ne contient que 2 messages
///					-Renboyer 1 dans le cas contraire
///  Le numéro des messages commencent à 1 , donc si on met num_msg=0 , la fonction renverra 0 ou bug
{
    int etat=1; //variable qui sera renvoyer , elle est a 1 par default et passe a 0 que dans les cas décrit en haut
    Lettre *Parcours=liste->premier;
	if(num_envoi_existant(liste,num_envoi)==0)//Si la lettre n'existe pas
	{
		etat=0;
		return etat;
	}
    while(Parcours!=NULL) //On va jusqu'a la fin de la liste
    {
		if(Parcours->num==num_envoi)//si parcours pointe sur la bonne lettre
			{
				if(Parcours->premiermsg==NULL)
				{
					etat=0;//Si on a aucun message dans la lettre
				}
				else
				{
					Message *ParcoursMsg=Parcours->premiermsg;
		            int i=0;
					while(ParcoursMsg!=NULL)
					{

					    if(i==num_msg-1)//Si parcoursMsg pointe sur le bon message
					    {
					        strcpy(buf,ParcoursMsg->msg);
					        break;
					    }
					    i++;
						ParcoursMsg=ParcoursMsg->suivant;
					}
				    if(ParcoursMsg==NULL)
				    {
				        etat=0;//si ParcoursMsg est égale a NULL ca veut dire que le num_msg ne corresponds à aucun message existant et qu'on est donc pas apssé dans le break
				    }
				}
			}
		Parcours=Parcours->suivant;

    }
    return(etat); //Si on est passé dans aucun des etat=0 , il est a 1 de base
}


void AddMessageLettre(char* message,ListeBal *liste,int num_envoi)///Fonction qui permet d'ajouter un message a une lettre avec comme id "num_envoi" /Messsage ajouter a la fin
{
	Lettre *Parcours=liste->premier;
    while(Parcours!=NULL) //On va jusqu'a la fin de la liste
    {
		if(Parcours->num==num_envoi)//si parcours pointe sur la bonne lettre
			{
				Message *NvMsg=calloc(1,sizeof(*NvMsg));//Création nouveau message et allocation mémoire

				if(Parcours->premiermsg==NULL)//Si il n'y avait pas de premier message
				{
					Parcours->premiermsg=NvMsg;//On l'ajoute
					strcpy(Parcours->premiermsg->msg,message);//On copie la chaine de caractere "message" dans la chaine de caractere de la structure message
					Parcours->premiermsg->suivant=NULL; //on initilaise a NULL le msg suivant
				}
				else
				{
					Message *ParcoursMsg=Parcours->premiermsg;
					while(ParcoursMsg->suivant!=NULL)//Parcours des message jusqu'aux dernier qui pointe sur un suivant NULL
					{

						ParcoursMsg=ParcoursMsg->suivant;
					}
					//ParcoursMSg pointe sur le dernier message
					ParcoursMsg->suivant=NvMsg; //on ajoute le nouveau message a la fin
				    ParcoursMsg=ParcoursMsg->suivant;//ParcoursMsg pointe alors sur le nouveau message
					strcpy(ParcoursMsg->msg,message);//Copie de la chaine de caractere message dans le nouveau message
					ParcoursMsg->suivant=NULL; //on initilaise a NULL le msg suivant
				}
			}
		Parcours=Parcours->suivant;
	}


}


void detection_numero_msg(char *message,int *num_stock)
///Fonction qui permet de detecter les numéro des msg qui sont contenue dans les 5 premiers caractere du message recu par le puit
///Retourne le numéro dans num_stock
{
	//Le numéro est forcément sur les 5 premier charactere
	char blanc='-';
	char temp_stock[6]; //chiffre max=99999 (5char)+1 char de fin \0
	int debut=0;
	for(int i=0;i<=4; i++)
	{
		if(message[i]==blanc)
		{
			continue; ///on continue (non-exécution du bas) si les caractere du messages sont = "-" ce qui permet d'ajouter dans temp_stock que les numéro du message
		}
		temp_stock[debut]=message[i];
		debut++;

	}
	temp_stock [debut+1]='\0';
	*num_stock=atoi(temp_stock);//convertie le numéro qui est en chaine de caractere en integer
}


int detection_e_ou_r(char *message)
///Fonction qui detecte si celui qui a envoyé une trame au -b et un -e ou un -r (Quand je suis en -r j'envoi une trame spéciale au -b alors qu'en -e j'envoi pas de trame spéciale)
///renvoie 0 si c'est -e , renvoie 1 si c'est -r
{
	///La trame spéciale que j'envoie est [reception]
	char mot[]="reception";
	int mode;
	for(int i=0;i<9; i++)
		{
			if(message[i]!=mot[i])
			{
				mode=0; //mode=0 veut dire que le client est celui qui veut envoie -e
				break;
			}
		mode=1; //mode =1 veut dire que le client est celui qui veut recevoir -r
		}

	return mode;
}

int num_reception_si_r(char *trame)
///Fonction qu'on utilise seulement si on est en -r et qui retourne le numéro de reception exemple -r2 la fonction retourne alors 2
///Fonctionne car la trame spéciale envoyé et utilisé dans detection_ou_r est enfaite reception'num' ex: reception2
{
	char test[6]={0};//init a zero de la chaine de caractere
	int debut=0;
		for(int i=9;;i++) //On sait que le num d'envoi est aprés 'reception' (programé comme ca) donc on prend les caractere aprés 9 et on les mets dans test[]
		{
			if(trame[i]=='\0')//si on est arrivé en fin de chaine(donc fin du numéro) on break;
			{
				test[debut]='\0';
				break;
			}
			test[debut]=trame[i];
			debut++;
		}
		return atoi(test);//on converti notre chaine de caractere en int
}


void construire_message(char *message, char motif, int lg,int num_msg)
{
	int i;
	char blanc='-';//
	char mot[6]={0};
	sprintf(mot, "%d", num_msg);//fonction qui permet de transformer un int en chaine de caractere (inverse de atoi)
	for (i=0;i<=4;i++)//5 premier caractere=numéro ex:----4
	{
		if(mot[i]=='\0')
		{
			message[i]=blanc;
			continue;
		}
		message[i] = mot[i];
	}
	for (i=5;i<lg;i++)//reste des caractere=lettre de "a" a "z" (motif) ex=----4aaaaaaaaa...(lg fois)
	{
			message[i] = motif;
	}
}



void afficher_message(char *message, int lg)///fonction qui permet d'afficher les lg caractere du message char * message
{
	int i;
	for (i=0;i<lg;i++)
	{
		printf("%c", message[i]);

	}
}

void main (int argc, char **argv)
{

	char *hostname;
	int sockfd;		/* socket */
	int sockfd_bis;
	int portno;		/* port to listen on */
	struct hostent *hp ;
	struct sockaddr_in serveraddr;
	struct sockaddr_in clientaddr;
	int lg_octet_lu; ///Nombre d'octet lu dans le read
	char buf[BUFSIZE]={0};
	int clientlen;
	int serverlen;
	int c;
	int n=0;
	int bal=0;//mode boite au lettre bal=0 si non-actif bal=1 si actif
	int reception=-1;///mode reception -r si non utilisé=-1 , sinon stock le num auquel on veut reception
	int envoie=-1;///mode envoie -e si non utilisé =-1 sinon stock le num auquel on veut envoyer
	extern char *optarg;
	extern int optind;
	int nb_message = -1; /* Nb de messages à envoyer ou à recevoir, par défaut : 10 en émission, infini en réception */
	int lg_message = -1;/* Taille des messages à envoyer ou à recevoir, par défaut : 30 en émission, infini en réception */
	int source = -1 ; /* 0=puits, 1=source */
	int protocol=-1; ///mettre UDP par défaut  protocol=1 ->TCP protocol=0->UDP
	int motif;///Char ascii de a
	int nb_max=5;///TCP dimensionnement
	////Variable pour le select
	fd_set rfds; /* ensemble des descripteur à tester en lecture */
	int retval;
	int n_sel ; ///stocke le max des sockets +1
	ListeBal *liste1=init();//init de liste1
	while ((c = getopt(argc, argv, "l:tun:spr:e:b")) != -1)
	{		/// ./tsockTCP.c -s -u -n50 port host
		switch (c) {
			case 'p':
				if (source == 1) {
					printf("usage: cmd [-p|-s][-n ##]\n");
					exit(1);
				}
				source = 0;
				break;

			case 's':
				if (source == 0) {
					printf("usage: cmd [-p|-s][-n ##]\n");
					exit(1) ;
				}
				source = 1;
				break;

			case 'u':
				if (protocol == 1) {
					printf("usage: cmd [-p|-s][-n ##]\n");
					exit(1) ;
				}
				protocol=0;
				break;

			case 't':
				if (protocol == 0) {
					printf("usage: cmd [-p|-s][-n ##]\n");
					exit(1) ;
				}
				protocol=1;
				break;

			case 'b':
				if (protocol == 0) {
						printf("usage: cmd -b port \n");
						exit(1) ;
					}
				protocol=1; ///Mode TCP actif
				source=0;///Mode puit
				bal = 1;///mode boite au lettre
				break;

			case 'e':
				protocol=1; ///Mode TCP actif
				source=1;///Mode source
				envoie = atoi(optarg);///atoi converti char en int	if (protocol == 1)///TCP socket
	{
		printf("On est en TCP\n");
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
	}
	else///UDP socket
	{
		printf("On est en UDP\n");
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	}

	if (source == -1 || protocol==-1) ///erreur pas de protocole ou de source initialisé
	{
		printf("usage: cmd [-p|-s][-n ##]\n");
		exit(1) ;
	}

				break;

			case 'r':
				protocol=1; ///Mode TCP actif
				source=1;///On se met en envoi au début puis on passe en réception
				reception = atoi(optarg);///atoi converti char en int
				break;

			case 'n':
				nb_message = atoi(optarg);///atoi converti char en int
				break;

			case 'l':
				lg_message = atoi(optarg);
				break;

			default:
				printf("usage: cmd [-p|-s][-n ##]\n");
				break;
		}
	}

	if (protocol == 1)///TCP socket
	{
		printf("On est en TCP\n");
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
	}
	else///UDP socket
	{
		printf("On est en UDP\n");
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	}

	if (source == -1 || protocol==-1) ///erreur pas de protocole ou de source initialisé
	{
		printf("usage: cmd [-p|-s][-n ##]\n");
		exit(1) ;
	}

	if (source == 1)///Mode Source
	{
		printf("On est dans le source\n");
		hostname = argv[optind];
		portno = atoi(argv[optind+1]);
		memset((char *)& serveraddr, 0, sizeof(serveraddr)) ;
		serveraddr.sin_family = AF_INET;
		serveraddr.sin_port = portno;
		hp = gethostbyname(hostname);
		if (hp == NULL)
		{
			fprintf(stderr,"ERROR, no such host as %s\n", hostname);
			exit(0);
		}
		memcpy((char*)&(serveraddr.sin_addr.s_addr),hp->h_addr,hp->h_length ) ;
	}
	else ///Mode Puit
	{
		printf("on est dans le puits\n");
		portno = atoi(argv[optind]);
		serveraddr.sin_family = AF_INET;
		serveraddr.sin_addr.s_addr = INADDR_ANY;
		serveraddr.sin_port = portno;
		if (bind(sockfd, (struct sockaddr *)&serveraddr,sizeof(serveraddr)) < 0)
		{
			perror("Error on binding");
			exit(1);
		}
	}


	if (nb_message != -1)
	{
		if (source == 1)
		{
			printf("nb de tampons à envoyer : %d\n", nb_message);
		}
		else
		{
			printf("nb de tampons à recevoir : %d\n", nb_message);
		}
	}
	else {
		if (source == 1)
		{
			nb_message = 10 ;
			printf("nb de tampons à envoyer = 10 par défaut\n");
		}
		else
		{
			nb_message=-1; ///on met a -1 pour dire qu'on n'a pas préciser de nombre de message et donc infini (utilisé plus tard dans un if)
			printf("nb de tampons à recevoir = infini\n");
		}
	}
	if (lg_message == -1) ///Taille defaut des messages
	{
		lg_message = 30 ;
	}

	if (source == 1) ///Source=1 =>Client
	{
		if(protocol==0)///CLIENT UDP
		{
			motif=97;///char ascii de a
			printf("lg_messg_emis=%d, port=%d, nb_envois=%d,Protocole=UDP, dest=%s\n",lg_message,portno,nb_message,hostname);
			for(int msg=1;msg<=nb_message;msg++)
			{
				construire_message(buf,(char)motif,lg_message,msg);
				printf("SOURCE: ENVOI n°%d (%d) [%s]\n",msg,lg_message,buf);
				n = sendto(sockfd,buf, strlen(buf), 0, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
				if (n ==-1)
				{
				  perror("ERROR in sendto");
				}
				motif++;
				if(motif>122)///char ascii de z
				{
					motif=97;
				}
			}
			printf("SOURCE: fin\n");
		}
		else if(envoie==-1 && reception==-1 && protocol==1)///CLIENT TCP NORMAL
		{
			if(connect(sockfd,(struct sockaddr*)&serveraddr, sizeof(serveraddr))==-1)
			{
				printf("Erreur connect()");
				exit(0);
			}
			motif=97;///char ascii de a
			printf("lg_messg_emis=%d, port=%d, nb_envois=%d,Protocole=TCP, dest=%s\n",lg_message,portno,nb_message,hostname);
			for(int msg=1;msg<=nb_message;msg++)
			{
				construire_message(buf,(char)motif,lg_message,msg);
				printf("SOURCE: ENVOI n°%d (%d) [%s]\n",msg,lg_message,buf);
				n = send(sockfd,buf, strlen(buf),0);
				if (n ==-1)
				{
					perror("ERROR in send");
				}
				motif++;
				if(motif>122)///char ascii de z
				{
					motif=97;
				}
			}
			printf("SOURCE: fin\n");
		}
		else if (envoie!=-1 &&reception==-1&&protocol==1) ///Client TCP en mode envoie BAL (Boite au Lettre) -e#
		{
			if(connect(sockfd,(struct sockaddr*)&serveraddr, sizeof(serveraddr))==-1)
			{
				printf("Erreur connect()");
				exit(0);
			}
			motif=97;///char ascii de a
			printf("lg_messg_emis=%d, port=%d, nb_envois=%d,Protocole=TCP, dest=%s\n",lg_message,portno,nb_message,hostname);
			for(int msg=1;msg<=nb_message;msg++)
			{
				construire_message(buf,(char)motif,lg_message,envoie);
				printf("SOURCE: Envoi lettre n°%d à destination du recepteur %d (%d) [%s]\n",msg,envoie,lg_message,buf);
				n = send(sockfd,buf, strlen(buf),0);
				if (n ==-1)
				{
					perror("ERROR in send");
				}
				motif++;
				if(motif>122)///char ascii de z
				{
					motif=97;
				}
			}
			printf("SOURCE: fin\n");
		}
		else if (envoie==-1 &&reception!=-1&&protocol==1) ///Client TCP mode reception -r#
		{
			char num_reception[5]={0};
			char mot[]="reception";
			char trame[20]={0};
			///envoie de la trame reception+num
			printf("Client reception mdoe\n");
			if(connect(sockfd,(struct sockaddr*)&serveraddr, sizeof(serveraddr))==-1)
			{
				printf("Erreur connect()");
				exit(0);
			}
			sprintf(num_reception, "%d", reception);
			strcat(trame,mot);
			strcat(trame,num_reception);
			printf("RECEPTION : Demande de récupération de ses lettres par  le récepteur %d,port=%d, TP=TCP, dest=%s\n",reception,portno,hostname);
			n = send(sockfd,trame, strlen(trame),0);
			if (n ==-1)
			{
				perror("ERROR in send");
			}
			///On passe en mode puit -r#
			int msg=1;
			while(1)
			{
				if ((lg_octet_lu = read(sockfd, buf,lg_message)) < 0)
				{
					printf("échec du read\n") ;
					exit(1) ;
				}
				if(lg_octet_lu==0)///En gros read va lire le flux de donnée en se décalant de lg_message a chaque appel et au bout d'un momment il aura 0"
				{
					printf("RECEPTION:fin \n");
					break;
				}
				printf("RECEPTION : Récupération lettre par le récepteur n°%d (%d) [",msg,lg_message);
				afficher_message(buf,lg_octet_lu);
				printf("]\n");
				msg++;
			}
		}

	}
	else ///MODE Serveur

		if(protocol==0 ) ///SERVEUR UDP Normal
		{
			clientlen=sizeof(clientaddr);
			if(nb_message==-1) ///Fonctionnement UDP->Infinie
			{
				printf("lg_messg_lu=%d, port=%d, nb_receptions=infini,Protocole=UDP\n",lg_message,portno);
				int msg=1;
				while(1)
				{
					n = recvfrom(sockfd, buf, BUFSIZE, 0,(struct sockaddr *)&clientaddr, &clientlen);
					printf("PUIT: Reception n°%d (%d) [",msg,lg_message);
					afficher_message(buf,lg_message);
					printf("]\n");
					if(n==-1)
					{
						printf("ERREUR");
						exit(0);
					}
					msg++;
				}
			}
			else	///Fonctionnement UPD ->NB_MESSAGE
			{
				printf("lg_messg_lu=%d, port=%d, nb_receptions=%d,Protocole=UDP\n",lg_message,portno,nb_message);
				int msg=1;
				while(msg<=nb_message)
				{
					n = recvfrom(sockfd, buf, BUFSIZE,0,(struct sockaddr *)&clientaddr, &clientlen);
					printf("PUIT: Reception n°%d (%d) [",msg,lg_message);
					afficher_message(buf,lg_message);
					printf("]\n");
					if(n==-1)
					{
						printf("ERREUR");
						exit(0);
					}
					msg++;
				}
			}
		}
		else if(bal==0 && protocol==1)	///Serveur TCP Normal
		{
			printf("lg_messg_lu=%d, port=%d, nb_receptions=infini,Protocole=TCP\n",lg_message,portno);
			clientlen=sizeof(clientaddr);
			if ((listen(sockfd, 5)) != 0)
			{
				printf("Listen failed...\n");
				exit(0);
			}
			if(nb_message==-1) ///TCP Fonctionnement infinie
			{
				while(1)
				{
					int msg=1;
					if ((sockfd_bis = accept( sockfd,(struct sockaddr *)&clientaddr,&clientlen)) == -1)
					{
						printf("échec du accept\n") ;
						exit(1) ;
					}
					switch (fork())
					{
						case - 1 :
						printf("erreur fork\n") ;
						exit(1) ;

						case 0 : /* on est dans le proc. fils */
						close(sockfd) ; /* fermeture socket du proc. père */
						while(1)
						{
							if ((lg_octet_lu = read(sockfd_bis, buf,lg_message)) < 0)
							{
								printf("échec du read\n") ;
								exit(1) ;
							}

							if(lg_octet_lu==0)///En gros read va lire le flux de donnée en se décalant de lg_message a chaque appel et au bout d'un momment il aura 0"
							{
								break;
							}
							printf("PUIT: Reception n°%d (%d) [",msg,lg_message);
							afficher_message(buf,lg_octet_lu);
							printf("]\n");
							msg++;
						}
						exit(0) ;

						default : /* on est dans le proc. père */
						close(sockfd_bis) ; /* fermeture socket du proc. fils */
					}
				}
			}
			else	///TCP FONCTIONNEMENT NB_MESSAGE
			{
				if ((sockfd_bis = accept( sockfd,(struct sockaddr *)&clientaddr,&clientlen)) == -1)
				{
					printf("échec du accept\n") ;
					exit(1) ;
				}
				int msg=1;
				while(msg<=nb_message)
				{


					if ((lg_octet_lu = read(sockfd_bis, buf,lg_message)) < 0)
					{
						printf("échec du read\n") ;
						exit(1) ;
					}
					if(lg_octet_lu==0)///En gros read va lire le flux de donnée en se décalant de lg_message a chaque appel et au bout d'un momment il aura 0"
					{

						if ((sockfd_bis = accept( sockfd,(struct sockaddr *)&clientaddr,&clientlen)) == -1)
						{
							printf("échec du accept\n") ;
							exit(1) ;
						}
					}
					printf("PUIT: Reception n°%d (%d) [",msg,lg_octet_lu);
					afficher_message(buf,lg_message);
					printf("]\n");
					msg++;
				}
			}
		}
		else ///mode boite au lettre ./tsock -b 9000
		{
			int num_msg=1;
			printf("lg_messg_lu=%d, port=%d, nb_receptions=infini,Protocole=TCP\n",lg_message,portno);
			clientlen=sizeof(clientaddr);
			printf("On est BAL\n") ;
			pid_t process_id;
			if ((listen(sockfd, 5)) != 0)
			{
				printf("Listen failed...\n");
				exit(0);
			}
			n_sel = sockfd;
			FD_ZERO(&rfds);
			int mode;
			int deja_rentrer_dans_boucle;
			int msg=0;
			while(1)
			{
				FD_SET(sockfd, &rfds);
				retval = 0;
				retval = select(n_sel+1, &rfds, NULL, NULL, NULL);
				if (retval< 0)
				{
					printf("%d \n",retval);
					perror("select() erreur");
					exit(-1);
				}
				else if (retval > 0)
				{
					printf("\nDonnées disponibles\n");  /* Note : FD_ISSET(0, &rfds) est vrai */
					if(FD_ISSET(sockfd, &rfds)) /* des données sont disponibles sur sock1*/
						{
							if ((sockfd_bis = accept( sockfd,(struct sockaddr *)&clientaddr,&clientlen)) == -1)
							{
								printf("échec du accept\n") ;
								exit(1) ;
							}
							if ((lg_octet_lu = read(sockfd_bis, buf,lg_message)) < 0)
							{
								printf("échec du read\n") ;
								exit(1) ;
							}
							if(lg_octet_lu==0)
							{
								close(sockfd_bis);
								break;
							}
							mode=detection_e_ou_r(buf);
							deja_rentrer_dans_boucle=0;
							while(1)
							{
								if(mode==0)
								{
									if (deja_rentrer_dans_boucle==1) ///Permet de rentrer tous le temps dans ce if sauf au premier passage dans cette boucle
									{
										if ((lg_octet_lu = read(sockfd_bis, buf,lg_message)) < 0)
										{
											printf("échec du read\n") ;
											exit(1) ;
										}
										if(lg_octet_lu==0)
										{
											close(sockfd_bis);
											break;
										}
									}
									detection_numero_msg(buf,&num_msg); ///detecte le numéro du message et le met dans num_msg
									printf("PUIT: Reception et stockage lettre n°%d pour le récepteur n°%d (%d) [",msg+1,num_msg,lg_message);
									afficher_message(buf,lg_octet_lu);
									printf("]\n");
									AddLettreFin(liste1,num_msg);
									AddMessageLettre(buf,liste1,num_msg);
									deja_rentrer_dans_boucle=1;
									msg++;
								}
								else if(mode==1)
								{
									int num_reception;
									num_reception=num_reception_si_r(buf);
									char bufer[50]={0};
									printf("\nEnvoi message au recepteur n°%d\n",num_reception);
									int i=1;
									///LectureMessage(liste1,num_reception);
									while(1)
									{
										if(retourmsg(bufer,liste1,num_reception,i)==0)
										{
											break;
										}
										n = send(sockfd_bis,bufer, strlen(bufer),0);
										if (n ==-1)
										{
											perror("ERROR in send");
										}
										i++;
									}
									printf("Envoi terminer\n");
									printf("Effacage en cours des messages envoyer....\n");
									EffacerMessage(liste1,num_reception);
									printf("Fin effacage\n");
									close(sockfd_bis);
									break;
								}

							}
						}
				}
			}
		}
}
