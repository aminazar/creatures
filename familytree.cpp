#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <fstream>
#include "animat.h"
#define TAB char(9)


class Generation;
class GenerationSet: public vector<Generation *>
{
 public:
    Generation *getGen(int gen);
    void print(ostream &out);
    void HTML_print(ostream &out);

    ~GenerationSet()
    {
        while(!empty())
        {
            delete back();
            pop_back();
        }
    }
}genList;

class node;

class Generation: public vector<node *>
{
    int gen;
public:
    Generation (int i){gen = i;}

    node *getNode(int num);
    
    int getGen(){return gen;}

    void print(ostream &out,int startGen);
    void HTML_print(ostream &out,int startGen);

    ~Generation()
    {
        while(!empty())
        {
            delete back();
            pop_back();
        }
    }
    friend class GenerationSet;
};

class node
{
    Generation *gen;
    int num;
    int children;
    node *p1,*p2;
    double score;
    bool is_printed;
    void add_children()
    {
        children++;
        if(p1)p1->add_children();
        if(p2)p2->add_children();
    }

public:
    node()
    {
        p1=p2=0;
        children=0;
    }

    node(int genr, int n)
    {
        children=0;
        p1=p2=0;
        Animat a;
        a.read(genr,n);
        score = a.avg_score;
        num = n;
        gen = genList.getGen(genr);

        if(genr)
        {
            Generation *g = genList.getGen(genr-1);
            string s1(a.parent[0]),s2(a.parent[1]);
            p1 = g->getNode(atoi(s1.substr(s1.find('-')+1).c_str()));
            p2 = g->getNode(atoi(s2.substr(s2.find('-')+1).c_str()));
        }
    }

    void preprint()
    {
        is_printed=false;
        if(p1)p1->preprint();
        if(p2)p2->preprint();
    }

    int print(ostream &out, vector<int> *spaces=0)
    {
        
        if(!spaces)spaces=new vector<int>;

        stringstream ostr;
        
        ostr<<"G:"<<gen->getGen()<<"-N:"<<num<<"-S:"<<score;
        out<<ostr.str();
        int l = ostr.str().length();
        if(!is_printed)
        {
           int enters=1;
           if(p1)
           {
               out<<"___";
               spaces->push_back(l);
               enters += p1->print(out,spaces) - 1;
               p1->is_printed=true;
           }
           if(p2)
           {
               for(int i=0;i<enters;i++)
               {
                   out<<"\n";
                   for(int j=0;j<spaces->size();j++)
                   {
                       for(int k=0;k<spaces->at(j);k++)
                           out<<' ';
                       out<<"|";
                       if(j!=spaces->size()-1)out<<"  ";
                   }                  
                   
               }
               out<<"__";
               enters += p2->print(out,spaces)-1;
               spaces->pop_back();
               p2->is_printed=true;
           }
           is_printed=true;
                      
           return enters;
        }
        else
            out<<"__^";
        return 1;
    }
    friend class Generation;
};

int main (int argc, char **argv)
{
    if(argc<3)exit(-1);
    int gen=atoi(argv[1]),num=atoi(argv[2]);
    node n(gen,num);
    ofstream TEXT_file;
    TEXT_file.open("test.txt",ios::out);
    if(TEXT_file.is_open())
    {
        n.preprint();
        n.print(TEXT_file);
        genList.print(TEXT_file);
    }
    ofstream HTML_file;
    HTML_file.open("test.html",ios::out);
    if(HTML_file.is_open())
        genList.HTML_print(HTML_file);
}

Generation *GenerationSet::getGen(int gen)
{
    for(iterator i=begin();i<end();i++)
        if((*i)->gen==gen)
            return *i;
    Generation *g = new Generation(gen);
    push_back(g);
    return g;
}

void GenerationSet::print(ostream &out)
{
    for(iterator i=begin();i<end();i++)
        (*i)->print(out,(*begin())->gen);
    
}

void GenerationSet::HTML_print(ostream &out)
{
    out<<"<table style=\"border:0;margin:0;padding:0;width:3000px;font-size:12px;font-family:sans-serif;border-collapse:collapse;\">\n";
    for(int i=size()-1;i>=0;i--)
        at(i)->HTML_print(out,(*begin())->gen);
    out<<"</table>";
}

node *Generation::getNode(int num)
{

    for(iterator i=begin();i<end();i++)
        if((*i)->num==num)
        {
            (*i)->add_children();
            return *i;
        }
    node *n = new node(gen,num);
    n->children++;
    push_back(n);
    return n;
}

void Generation::print(ostream &out,int startGen)
{
    unsigned long t=1;
    t<<=(startGen-gen);
    out<<"\nGeneration "<<gen<<endl;
    for(iterator i=begin();i<end();i++)
        out<<(*i)->num<<TAB<<(*i)->score<<TAB<<(double)((*i)->children)/t<<endl;
    
}

void Generation::HTML_print(ostream &out,int startGen)
{
    unsigned long t=1;
    t<<=(startGen-gen);
    out<<"<tr><td>\n";
    out<<"<table style=\"border:0;margin:0;padding:0;width:3000px;font-size:12px;font-family:sans-serif;color:white;border-collapse:collapse;\"><tr>\n";
    for(int i=0;i<size();i++)
    {
        double percentage = (double)(100*at(i)->children)/t;
        out<<"<td ";
        //out<<"colspan=\""<<at(i)->children*(1<<gen)<<"\"";
        out<<" style=\"padding:0;margin:0;border:1px solid white;width:"<<(int)(percentage*30)<<"px;";
        out<<"background:rgb("<<sto.IRandomX(0,15)*8<<","<<sto.IRandomX(0,15)*8<<","<<sto.IRandomX(0,15)*8<<")\">";
        out<<at(i)->num<<": "<<at(i)->score<<", "<<percentage<<'%';
        out<<"</td>\n";
    }
    out<<"</tr></table></td></tr>\n\n";
}