package com.example.raymarine;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.Drawable;
import android.text.TextPaint;
import android.util.AttributeSet;
import android.view.View;

import androidx.core.content.ContextCompat;
public class view_settings extends View {

    int couleurTrait, couleurTiret1, couleurTiret2, couleurTexte;
    private TextPaint mTextPaint;
    public view_settings(Context context) {
        super(context);
        init(null, 0);
    }
    public view_settings(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(attrs, 0);
    }
    public view_settings(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(attrs, defStyle);
    }
    private void init(AttributeSet attrs, int defStyle) {
        TypedArray a = getContext().getTheme().obtainStyledAttributes(
                attrs,
                R.styleable.view_settings,
                0, 0);


        try {
            couleurTrait  = a.getColor(R.styleable.view_settings_couleurTrait, 0);
            couleurTiret1 = a.getColor(R.styleable.view_settings_couleurTiret1, 0);
            couleurTiret2 = a.getColor(R.styleable.view_settings_couleurTiret2, 0);
            couleurTexte = a.getColor(R.styleable.view_settings_couleurTexte, 0);
            //setHeading(currentHeading);
        } finally {
            a.recycle();
        }

        // Set up a default TextPaint object
        mTextPaint = new TextPaint();
        mTextPaint.setFlags(Paint.ANTI_ALIAS_FLAG);
        mTextPaint.setTextAlign(Paint.Align.LEFT);


    }



    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // TODO: consider storing these as member variables to reduce
        // allocations per draw cycle.
        int paddingLeft = getPaddingLeft();
        int paddingTop = getPaddingTop();
        int paddingRight = getPaddingRight();
        int paddingBottom = getPaddingBottom();

        int contentWidth = getWidth() - paddingLeft - paddingRight;
        int contentHeight = getHeight() - paddingTop - paddingBottom;

        // Draw the text.


        mTextPaint.setStrokeWidth(5f);
        mTextPaint.setColor(couleurTrait);
        //canvas.drawLine((contentWidth/2)-250,100f, (contentWidth/2)+250, 100f,mTextPaint);

        canvas.drawLine((contentWidth/2)-250,100f, (contentWidth/2)+250, 100f,mTextPaint);


        mTextPaint.setStrokeWidth(5f);
        float lengthTiret=0f;
        for (float i = 0; i < 25; i++)
        {

            if (i % 5 == 0)
                {
                    mTextPaint.setColor(couleurTiret1);
                    lengthTiret = 60f;
                    mTextPaint.setStrokeWidth(5f);
                }
            else
            {
                mTextPaint.setColor(couleurTiret2);
                lengthTiret=40f;
                mTextPaint.setStrokeWidth(6f);
            }
            canvas.drawLine((contentWidth/2)-250+(i*20f),100f,(contentWidth/2)-250+(i*20f),100f-lengthTiret,mTextPaint);

            if (i % 10 == 0)
            {
                mTextPaint.setColor(couleurTexte);
                mTextPaint.setTextSize(50);
                mTextPaint.setTextAlign(Paint.Align.CENTER);
                canvas.drawText(String.valueOf(i+180),(contentWidth/2)-250+(i*20f),180f,mTextPaint);
            }
        }

    }


    public int getCouleurTrait() {
        return couleurTrait;
    }

    public void setCouleurTrait(int couleurTrait) {
        this.couleurTrait = couleurTrait;
        invalidate();
    }

    public int getCouleurTiret1() {
        return couleurTiret1;
    }

    public void setCouleurTiret1(int couleurTiret1) {
        this.couleurTiret1 = couleurTiret1;
        invalidate();
    }

    public int getCouleurTiret2() {
        return couleurTiret2;
    }

    public void setCouleurTiret2(int couleurTiret2) {
        this.couleurTiret2 = couleurTiret2;
        invalidate();
    }

    public int getCouleurTexte() {
        return couleurTexte;
    }

    public void setCouleurTexte(int couleurTexte) {
        this.couleurTexte = couleurTexte;
        invalidate();

    }
}